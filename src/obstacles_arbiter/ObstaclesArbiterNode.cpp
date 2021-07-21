/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

ObstaclesArbiterNode::ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag,
                                           std::unique_ptr<IPolygonCollisionDetection> collision_detection)
  : nh_{ nh }, tf_listener_{ tf_buffer_ }, log_tag_{ log_tag }, collision_detection_{ std::move(collision_detection) }
{
  detection_status_.status = detection_status_.UNKNOWN;

  safety_zone_length_ = ros_common_functions::getRosParam<double>("~obstacles_arbiter/safety_zone_length");
  safety_zone_width_ = ros_common_functions::getRosParam<double>("~obstacles_arbiter/safety_zone_width");

  objects_target_frame_id_ = ros_common_functions::getRosParam<std::string>("~objects_target_frame_id");
  publish_processing_time_ = ros_common_functions::getRosParam<bool>("~publish_processing_time");
  print_processing_time_ = ros_common_functions::getRosParam<bool>("~print_processing_time");

  // Initialize publishers and subscribers
  out_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("out_polygons", 1, true);
  path_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("path_polygons", 1, true);
  objects_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("objects_polygons", 1, true);
  detection_status_pub_ = nh->advertise<agribot_msgs::ObstacleDetectionStatus>("detection_status", 1, true);
  left_row_path_pub_ = nh->advertise<nav_msgs::Path>("left_row_path", 1, true);
  right_row_path_pub_ = nh->advertise<nav_msgs::Path>("right_row_path", 1, true);

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
    computeRowsPaths();

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

  // Publish status after collision checking. Mainly to handle NO_OBSTACLE situation
  publishDetectionStatus();

  // Publish output polygons for visualization
  publishOutputPolygons();
}

void ObstaclesArbiterNode::computeRowsPaths()
{
  nav_msgs::Path left_row_path;
  nav_msgs::Path right_row_path;
  left_row_path.header.frame_id = "base_footprint";
  left_row_path.header.stamp = ros::Time::now();
  right_row_path.header.frame_id = "base_footprint";
  right_row_path.header.stamp = ros::Time::now();

  agribot_msgs::ObjectPolygon left_row_polygon;
  agribot_msgs::ObjectPolygon right_row_polygon;

  if (objects_polygons_->polygons.size() < 2)
  {
    left_row_path.poses.clear();
    right_row_path.poses.clear();
  }
  else
  {
    // auto rows_polygons
    auto area_comparison = [](const agribot_msgs::ObjectPolygon& p1, const agribot_msgs::ObjectPolygon& p2) {
      return p1.area < p2.area;
    };

    // Sort objects in descending order by area
    std::sort(objects_polygons_->polygons.rbegin(), objects_polygons_->polygons.rend(), area_comparison);

    if (objects_polygons_->polygons[0].centroid.y > objects_polygons_->polygons[1].centroid.y)
    {
      left_row_polygon = objects_polygons_->polygons[0];
      right_row_polygon = objects_polygons_->polygons[1];
    }
    else
    {
      left_row_polygon = objects_polygons_->polygons[1];
      right_row_polygon = objects_polygons_->polygons[0];
    }
  }

  pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // Prepare segmentation object
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.1);
  seg.setMaxIterations(300);

  // Do segmentation
  pcl::PCLPointCloud2 left_row_pcl_pointcloud2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr left_row_pcl_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::ModelCoefficients left_line_coefficients;

  pcl_conversions::toPCL(left_row_polygon.point_cloud, left_row_pcl_pointcloud2);
  pcl::fromPCLPointCloud2(left_row_pcl_pointcloud2, *left_row_pcl_pointcloud);

  seg.setInputCloud(left_row_pcl_pointcloud);
  seg.segment(*inliers, left_line_coefficients);

  auto left_line_a = left_line_coefficients.values[3];
  auto left_line_b = left_line_coefficients.values[4];

  double left_line_start_x = 1.0;
  double left_line_end_x = 5.0;

  auto left_line_start_y = left_line_a * left_line_start_x + left_line_b;
  auto left_line_end_y = left_line_a * left_line_end_x + left_line_b;

  geometry_msgs::PoseStamped left_start_pose;
  left_start_pose.header = left_row_path.header;
  left_start_pose.pose.position.x = left_line_start_x;
  left_start_pose.pose.position.y = left_line_start_y;
  left_start_pose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped left_end_pose;
  left_end_pose.header = left_row_path.header;
  left_end_pose.pose.position.x = left_line_end_x;
  left_end_pose.pose.position.y = left_line_end_y;
  left_end_pose.pose.orientation.w = 1;

  left_row_path.poses.push_back(left_start_pose);
  left_row_path.poses.push_back(left_end_pose);

  geometry_msgs::PoseStamped right_start_pose;
  right_start_pose.header = right_row_path.header;
  right_start_pose.pose.position = Conversions::convertToPointMsg(right_row_polygon.centroid);
  right_start_pose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped right_end_pose;
  right_end_pose.header = right_row_path.header;
  right_end_pose.pose.position = Conversions::convertToPointMsg(right_row_polygon.centroid);
  right_end_pose.pose.position.x += 1.0;
  right_end_pose.pose.orientation.w = 1;

  right_row_path.poses.push_back(right_start_pose);
  right_row_path.poses.push_back(right_end_pose);

  left_row_path_pub_.publish(left_row_path);
  right_row_path_pub_.publish(right_row_path);
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
