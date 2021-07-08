/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

ObstaclesArbiterNode::ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag)
  : nh_{ nh }, tf_listener_{ tf_buffer_ }, log_tag_{ log_tag }
{
  detection_status_.status = detection_status_.UNKNOWN;

  safety_zone_length_ = ros_common_functions::getRosParam<float>("~obstacles_arbiter/safety_zone_length");
  safety_zone_width_ = ros_common_functions::getRosParam<float>("~obstacles_arbiter/safety_zone_width");

  objects_target_frame_id_ = ros_common_functions::getRosParam<std::string>("~objects_target_frame_id");
  publish_processing_time_ = ros_common_functions::getRosParam<bool>("~publish_processing_time");
  print_processing_time_ = ros_common_functions::getRosParam<bool>("~print_processing_time");

  // Initialize publishers and subscribers
  out_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("out_polygons", 1, true);
  path_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("path_polygons", 1, true);
  objects_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("objects_polygons", 1, true);
  detection_status_pub_ = nh->advertise<agribot_msgs::ObstacleDetectionStatus>("detection_status", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_object_array_sub_ = nh->subscribe("in_objects", 1, &ObstaclesArbiterNode::object3DArrayCallback, this);
  path_sub_ = nh->subscribe("path", 1, &ObstaclesArbiterNode::pathCallback, this);
}

void ObstaclesArbiterNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  try
  {
    timer.start();

    if (msg->objects.size() == 0)
    {
      objects_polygons_->polygons.clear();
    }
    else
    {
      if (msg->objects[0].header.frame_id != objects_target_frame_id_)
      {
        objects_polygons_ = Conversions::convertToObjectPolygonArray(
            Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        objects_polygons_ = Conversions::convertToObjectPolygonArray(*msg);
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
          Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_), safety_zone_length_, safety_zone_width_);
    }
    else
    {
      path_polygons_ = Conversions::convertToObjectPolygonArray(*msg, safety_zone_length_, safety_zone_width_);
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
  if (path_polygons_ == std::nullopt)
  {
    detection_status_.status = detection_status_.WAITING_FOR_PATH;
    publishDetectionStatus();

    ROS_WARN_STREAM_THROTTLE(5.0, log_tag_ << "Missing path info. Skipping calculation of output polygons...");
    return;
  }

  if (objects_polygons_ == std::nullopt)
  {
    detection_status_.status = detection_status_.WAITING_FOR_OBJECTS;
    publishDetectionStatus();

    ROS_WARN_STREAM_THROTTLE(5.0, log_tag_ << "Missing objects info. Skipping calculation of output polygons...");
    return;
  }

  // Clear detection status before checking new collisions
  detection_status_.status = detection_status_.NO_OBSTACLE;

  // If there are no polygons in any of the lists, NO_OBSTACLE will be published as for loop will be omitted
  for (auto& path_polygon : path_polygons_->polygons)
  {
    for (auto& object_polygon : objects_polygons_->polygons)
    {
      if (collision_detection_.areColliding(object_polygon, path_polygon))
      {
        detection_status_.status = detection_status_.OBSTACLE_DETECTED;
        publishDetectionStatus();

        // Mark path and objects polygons with _COLLIDING type
        object_polygon.type = object_polygon.OBSTACLE_COLLIDING;
        path_polygon.type = object_polygon.PATH_COLLIDING;
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
  output_polygons_.polygons.reserve(objects_polygons_->polygons.size() + path_polygons_->polygons.size());
  output_polygons_.polygons.insert(output_polygons_.polygons.end(), objects_polygons_->polygons.begin(),
                                   objects_polygons_->polygons.end());
  output_polygons_.polygons.insert(output_polygons_.polygons.end(), path_polygons_->polygons.begin(),
                                   path_polygons_->polygons.end());

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
