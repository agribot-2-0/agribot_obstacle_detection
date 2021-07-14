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
#include "agribot_obstacle_detection/point_cloud_late_fusion/PointCloudLateFusionNodeConfig.h"
#include "agribot_obstacle_detection/point_cloud_late_fusion/IntersectionOverUnion.h"

class PointCloudLateFusionNode
{
private:
  std::optional<agribot_msgs::Object3DArray> in_1_object_array_;
  std::optional<agribot_msgs::Object3DArray> in_2_object_array_;

  LateFusionMethod fusion_method_ = LateFusionMethod::rewrite_both;
  double iou_threshold_ = 0.0;

  std::string objects_target_frame_id_;
  bool publish_processing_time_;
  bool print_processing_time_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber in_1_object_array_sub_;
  ros::Subscriber in_2_object_array_sub_;
  ros::Publisher out_object_array_pub_;

  // Processing time publisher
  ros::Publisher processing_time_pub_;

  // TF Buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  dynamic_reconfigure::Server<agribot_obstacle_detection::PointCloudLateFusionConfig> dynamic_reconfigure_server_;

  // Logging and debug
  std::string log_tag_;
  ProcessingTimer timer;

  void objectArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg, int id);
  agribot_msgs::Object3DArray objectArrayFusion();
  agribot_msgs::Object3DArray intersectionOverUnionFusion();
  void publishOutObjectArrary(const agribot_msgs::Object3DArray& msg);
  void dynamicReconfigureCallback(const agribot_obstacle_detection::PointCloudLateFusionConfig& dynamic_config,
                                  uint32_t level);

public:
  PointCloudLateFusionNode(ros::NodeHandle* nh, const std::string& log_tag);
  ~PointCloudLateFusionNode() = default;
  void run() const;
};
