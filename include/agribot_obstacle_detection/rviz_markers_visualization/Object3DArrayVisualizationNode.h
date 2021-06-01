/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>

#include <string>

#include "agribot_common/CommonFunctions/ros_common_functions.h"
#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"

class Object3DArrayVisualizationNode
{
private:
  ros::NodeHandle* nh_;
  ros::Subscriber object3d_array_sub_;

  std::string target_frame_id_;
  std::string marker_namespace_;
  rviz_visual_tools::colors marker_color_;

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;
  void object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
  void updateMarkers(const agribot_msgs::Object3DArray::ConstPtr& msg);
  Eigen::Isometry3d getObjectEigenPose(const agribot_msgs::Object3D& object) const;

public:
  explicit Object3DArrayVisualizationNode(ros::NodeHandle* nh);
  ~Object3DArrayVisualizationNode();
  void run(void) const;
};
