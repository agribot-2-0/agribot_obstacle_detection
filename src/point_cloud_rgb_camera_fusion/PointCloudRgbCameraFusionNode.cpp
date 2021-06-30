/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/point_cloud_rgb_camera_fusion/PointCloudRgbCameraFusionNode.h"

PointCloudRgbCameraFusionNode::PointCloudRgbCameraFusionNode(ros::NodeHandle* nh, const std::string& log_tag)
  : nh_(nh), tf_listener_(tf_buffer_), log_tag_(log_tag)
{
  // Initialize publishers and subscribers
  out_object_array_pub_ = nh->advertise<agribot_msgs::Object3DArray>("out_objects", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_object_array_sub_ =
      nh->subscribe("in_3d_objects", 1, &PointCloudRgbCameraFusionNode::object3DArrayCallback, this);
}

void PointCloudRgbCameraFusionNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  // TODO(olek) - add fusion
  // For now, just rewrite objects
  out_object_array_pub_.publish(msg);
}

void PointCloudRgbCameraFusionNode::run() const
{
  ros::spin();
}
