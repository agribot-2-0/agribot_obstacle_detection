/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>

#include <optional>
#include <string>

#include "agribot_common/CommonFunctions/ros_common_functions.h"

#include "agribot_obstacle_detection/obstacles_arbiter/collision_detection/SatPolygonCollisionDetection.h"
#include "agribot_obstacle_detection/util/ProcessingTimer.h"
#include "agribot_obstacle_detection/util/Conversions.h"

#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"
#include "agribot_msgs/ObjectPolygonArray.h"
#include "agribot_msgs/ObstacleDetectionStatus.h"

class ObstaclesArbiterNode
{
private:
  float safety_zone_length_;
  float safety_zone_width_;
  std::string objects_target_frame_id_;
  bool publish_processing_time_;
  bool print_processing_time_;

  std::optional<agribot_msgs::ObjectPolygonArray> path_polygons_;
  std::optional<agribot_msgs::ObjectPolygonArray> objects_polygons_;
  agribot_msgs::ObjectPolygonArray output_polygons_;
  agribot_msgs::ObstacleDetectionStatus detection_status_;

  SatPolygonCollisionDetection collision_detection_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber in_object_array_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher out_polygons_pub_;
  ros::Publisher path_polygons_pub_;
  ros::Publisher objects_polygons_pub_;
  ros::Publisher detection_status_pub_;

  // Processing time publisher
  ros::Publisher processing_time_pub_;

  // TF Buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Logging and debug
  std::string log_tag_;
  ProcessingTimer timer;

    void object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void checkCollisions();
    void publishOutputPolygons();
    void publishDetectionStatus();
    void sendProcessingTimeInfo(double time_ms);

public:
  ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag);
  void run() const;
};
