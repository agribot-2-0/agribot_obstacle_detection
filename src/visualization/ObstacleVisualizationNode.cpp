/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/visualization/ObstacleVisualizationNode.h"

ObstacleVisualizationNode::ObstacleVisualizationNode(ros::NodeHandle* _nh) : firstVisionDetectionMsg(true)

{
  nh.reset(_nh);
  visionDetectionSub = nh->subscribe("vision_detection", 1, &ObstacleVisualizationNode::visionDetectionCallback, this);
}

ObstacleVisualizationNode::~ObstacleVisualizationNode()
{
}

void ObstacleVisualizationNode::visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg)
{
  if (firstVisionDetectionMsg)
  {
    initVisionVisualTools(msg->header.frame_id);
    firstVisionDetectionMsg = false;
  }

  publishVisionMarkers(msg);
}

void ObstacleVisualizationNode::initVisionVisualTools(const std::string& frameId)
{
  visionVisualTools.reset(new rviz_visual_tools::RvizVisualTools(frameId, "vision_markers"));
}

void ObstacleVisualizationNode::publishVisionMarkers(const agribot_msgs::VisionDetection::ConstPtr& msg) const
{
  // Clear previous markers
  visionVisualTools->deleteAllMarkers();

  // Publish bounding box for each object
  for (const auto& object : msg->objects)
  {
    publishBoundingBoxMarker(object.box, object.id);
  }

  // Trigger visualization
  visionVisualTools->trigger(false);
}

void ObstacleVisualizationNode::publishBoundingBoxMarker(const agribot_msgs::BoundingBox& box, uint32_t id) const
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  Eigen::Vector3d minPoint, maxPoint;
  tf::pointMsgToEigen(box.min_point, minPoint);
  tf::pointMsgToEigen(box.max_point, maxPoint);

  visionVisualTools->publishWireframeCuboid(pose, minPoint, maxPoint, rviz_visual_tools::BLUE, "Obstacle");

  // Translation for text alignment
  pose.translation() = maxPoint;

  std::string stringId = "Obstacle_" + std::to_string(id);
  visionVisualTools->publishText(pose, stringId, rviz_visual_tools::WHITE, rviz_visual_tools::TEXTSIZE, false);
}

void ObstacleVisualizationNode::run() const
{
  ros::spin();
}
