/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

ObstaclesArbiterNode::ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag,
                                           std::unique_ptr<IPolygonCollisionDetection> collision_detection)
  : nh_{ nh }, tf_listener_{ tf_buffer_ }, log_tag_{ log_tag }, collision_detection_{ std::move(collision_detection) }
{
  detection_status_.status = detection_status_.UNKNOWN;
  objects_target_frame_id_ = "base_footprint";

  typename dynamic_reconfigure::Server<agribot_obstacle_detection::ObstaclesArbiterConfig>::CallbackType
      dynamic_reconfigure_function = boost::bind(&ObstaclesArbiterNode::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_function);

  // Initialize publishers and subscribers
  out_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("out_polygons", 1, true);
  path_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("path_polygons", 1, true);
  objects_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("objects_polygons", 1, true);
  detection_status_pub_ = nh->advertise<agribot_msgs::ObstacleDetectionStatus>("detection_status", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_object_array_sub_ = nh->subscribe("in_objects", 1, &ObstaclesArbiterNode::object3DArrayCallback, this);
  path_sub_ = nh->subscribe("path", 1, &ObstaclesArbiterNode::pathCallback, this);
}

agribot_msgs::ObjectPolygon ObstaclesArbiterNode::buildRobotPolygon()
{
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_footprint";
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.pose.orientation.w = 1.0;

  return Conversions::convertToObjectPolygon(robot_pose, robot_safety_zone_length_, robot_safety_zone_width_);
}

void ObstaclesArbiterNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  try
  {
    timer.start();

    if (msg->objects.size() == 0)
    {
      objects_polygons_ = agribot_msgs::ObjectPolygonArray();
    }
    else
    {
      if (msg->objects[0].header.frame_id != objects_target_frame_id_)
      {
        objects_polygons_ = Conversions::convertToConvexHullPolygonArray(
            Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        objects_polygons_ = Conversions::convertToConvexHullPolygonArray(*msg);
      }
    }

    checkCollisions();

    timer.stop();
    sendProcessingTimeInfo(timer.get_final_elapsed_time_ms());
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM(log_tag_ << e.what());
  }
}

void ObstaclesArbiterNode::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  try
  {
    timer.start();

    if (msg->header.frame_id != objects_target_frame_id_)
    {
      path_polygons_ = Conversions::convertToObjectPolygonArray(
          Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_), min_path_safety_zone_length_,
          min_path_safety_zone_width_, max_path_safety_zone_length_, max_path_safety_zone_width_,
          path_safety_zone_horizon_);
    }
    else
    {
      path_polygons_ = Conversions::convertToObjectPolygonArray(
          *msg, min_path_safety_zone_length_, min_path_safety_zone_width_, max_path_safety_zone_length_,
          max_path_safety_zone_width_, path_safety_zone_horizon_);
    }

    checkCollisions();

    timer.stop();
    sendProcessingTimeInfo(timer.get_final_elapsed_time_ms());
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM(log_tag_ << e.what());
  }
}

void ObstaclesArbiterNode::checkCollisions()
{
  if (objects_polygons_ == std::nullopt)
  {
    detection_status_.status = detection_status_.WAITING_FOR_OBJECTS;
    publishDetectionStatus();

    ROS_WARN_STREAM_THROTTLE(5.0, log_tag_ << "Missing objects info. Skipping calculation of output polygons...");
    return;
  }

  if (path_polygons_ == std::nullopt)
  {
    detection_status_.status = detection_status_.WAITING_FOR_PATH;
    publishDetectionStatus();

    ROS_WARN_STREAM_THROTTLE(5.0, log_tag_ << "Missing path info. Checking collisions only for robot polygon...");
  }

  // Clear detection status before checking new collisions
  detection_status_.status = detection_status_.NO_OBSTACLE;

  // Regenerate robot polygon before checking new collisions
  robot_polygon_ = buildRobotPolygon();
  robot_polygon_.type = robot_polygon_.PATH_CLEAR;

  // If there are no polygons in any of the lists, NO_OBSTACLE will be published as for loop will be omitted
  for (auto& object_polygon : objects_polygons_->polygons)
  {
    // Check collision with robot polygon
    if (collision_detection_->areColliding(object_polygon, robot_polygon_))
    {
      detection_status_.status = detection_status_.OBSTACLE_DETECTED;
      publishDetectionStatus();

      // Mark path and objects polygons with _COLLIDING type
      object_polygon.type = object_polygon.OBSTACLE_COLLIDING;
      robot_polygon_.type = object_polygon.PATH_COLLIDING;
    }

    // Check collisions with path polygons
    if (path_polygons_ != std::nullopt)
    {
      for (auto& path_polygon : path_polygons_->polygons)
      {
        if (collision_detection_->areColliding(object_polygon, path_polygon))
        {
          detection_status_.status = detection_status_.OBSTACLE_DETECTED;
          publishDetectionStatus();

          // Mark path and objects polygons with _COLLIDING type
          object_polygon.type = object_polygon.OBSTACLE_COLLIDING;
          path_polygon.type = object_polygon.PATH_COLLIDING;
        }
      }
    }
  }

  // Publish status after collision checking. Mainly to handle NO_OBSTACLE situation
  publishDetectionStatus();

  // Publish output polygons for visualization
  publishOutputPolygons();
}

void ObstaclesArbiterNode::publishOutputPolygons()
{
  // Just rewrite path and obstacles polygons. They should have proper "type" set already
  output_polygons_.polygons.clear();

  std::size_t output_size = objects_polygons_->polygons.size() + 1;  // At least for objects and robot polygon

  if (path_polygons_ != std::nullopt)
  {
    output_size += path_polygons_->polygons.size();
  }

  output_polygons_.polygons.reserve(output_size);

  if (path_polygons_ != std::nullopt)
  {
    output_polygons_.polygons.insert(output_polygons_.polygons.end(), path_polygons_->polygons.begin(),
                                     path_polygons_->polygons.end());
  }

  output_polygons_.polygons.insert(output_polygons_.polygons.end(), objects_polygons_->polygons.begin(),
                                   objects_polygons_->polygons.end());
  output_polygons_.polygons.push_back(robot_polygon_);

  out_polygons_pub_.publish(output_polygons_);
}

void ObstaclesArbiterNode::publishDetectionStatus()
{
  detection_status_pub_.publish(detection_status_);
}

void ObstaclesArbiterNode::sendProcessingTimeInfo(double time_ms)
{
  if (publish_processing_time_)
  {
    std_msgs::Float64 processing_time_msg;
    processing_time_msg.data = time_ms;
    processing_time_pub_.publish(processing_time_msg);
  }

  if (print_processing_time_)
  {
    ROS_INFO_STREAM(log_tag_ << "Full processing took:\t" << std::setprecision(2) << time_ms << " [ms]");
  }
}

void ObstaclesArbiterNode::run() const
{
  ros::spin();
}

void ObstaclesArbiterNode::dynamicReconfigureCallback(
    const agribot_obstacle_detection::ObstaclesArbiterConfig& dynamic_config, uint32_t level)
{
  static_cast<void>(level);  // Suppress unused variable warning

  publish_processing_time_ = dynamic_config.publish_processing_time;
  print_processing_time_ = dynamic_config.print_processing_time;
  robot_safety_zone_length_ = dynamic_config.robot_safety_zone_length;
  robot_safety_zone_width_ = dynamic_config.robot_safety_zone_width;
  min_path_safety_zone_length_ = dynamic_config.min_path_safety_zone_length;
  min_path_safety_zone_width_ = dynamic_config.min_path_safety_zone_width;
  max_path_safety_zone_length_ = dynamic_config.max_path_safety_zone_length;
  max_path_safety_zone_width_ = dynamic_config.max_path_safety_zone_width;
  path_safety_zone_horizon_ = dynamic_config.path_safety_zone_horizon;
}
