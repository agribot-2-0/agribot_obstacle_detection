/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>

#include <string>
#include <optional>

#include "agribot_common/CommonFunctions/ros_common_functions.h"
#include "agribot_msgs/ObjectPolygon.h"
#include "agribot_msgs/ObjectPolygonArray.h"

class ObjectPolygonArrayVisualizationNode
{
private:
  ros::NodeHandle* nh_;
  ros::Subscriber object_polygon_array_sub_;
  std::string marker_namespace_;
  rviz_visual_tools::colors unknown_marker_color_;
  rviz_visual_tools::colors obstacle_clear_marker_color_;
  rviz_visual_tools::colors obstacle_colliding_marker_color_;
  rviz_visual_tools::colors path_clear_marker_color_;
  rviz_visual_tools::colors path_colliding_marker_color_;
  std::optional<std::string> target_frame_id_;

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;
  void initRvizVisualTools();
  void objectPolygonArrayCallback(const agribot_msgs::ObjectPolygonArray::ConstPtr& msg);
  void updateMarkers(const agribot_msgs::ObjectPolygonArray::ConstPtr& msg);

public:
  explicit ObjectPolygonArrayVisualizationNode(ros::NodeHandle* nh);
  void run() const;
};
