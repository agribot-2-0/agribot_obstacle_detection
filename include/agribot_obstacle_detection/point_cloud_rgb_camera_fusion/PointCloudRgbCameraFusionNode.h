/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Float64.h>

#include <string>

#include "agribot_obstacle_detection/util/ProcessingTimer.h"

#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"
#include "agribot_obstacle_detection/PointCloudLateFusionConfig.h"

class PointCloudRgbCameraFusionNode
{
private:
  std::optional<agribot_msgs::Object3DArray> in_1_object_array_;
  std::optional<agribot_msgs::Object3DArray> in_2_object_array_;

  std::string objects_target_frame_id_;
  bool publish_processing_time_;
  bool print_processing_time_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber in_object_array_sub_;
  ros::Publisher out_object_array_pub_;

  // Processing time publisher
  ros::Publisher processing_time_pub_;

  // TF Buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Logging and debug
  std::string log_tag_;
  ProcessingTimer timer;

public:
  PointCloudRgbCameraFusionNode(ros::NodeHandle* nh, const std::string& log_tag);
  ~PointCloudRgbCameraFusionNode() = default;

  void object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
  void run() const;
};
