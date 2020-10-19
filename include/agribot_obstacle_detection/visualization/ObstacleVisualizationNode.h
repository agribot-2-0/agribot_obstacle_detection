/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <agribot_msgs/VisionDetection.h>
#include <agribot_msgs/DetectedObject.h>
#include <agribot_msgs/BoundingBox.h>

#include <eigen_conversions/eigen_msg.h>

#include <string>
#include <memory>

class ObstacleVisualizationNode
{
private:
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Subscriber visionDetectionSub;

  bool firstVisionDetectionMsg;

  // Rviz visualization
  rviz_visual_tools::RvizVisualToolsPtr visionVisualTools;

public:
  explicit ObstacleVisualizationNode(ros::NodeHandle* _nh);
  ~ObstacleVisualizationNode();

  void visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg);
  void initVisionVisualTools(const std::string& frameId);
  void publishVisionMarkers(const agribot_msgs::VisionDetection::ConstPtr& msg) const;
  void publishBoundingBoxMarker(const agribot_msgs::BoundingBox& box, uint32_t id) const;
  void run(void) const;
};
