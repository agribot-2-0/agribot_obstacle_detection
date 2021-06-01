/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/rviz_markers_visualization/Object3DArrayVisualizationNode.h"

Object3DArrayVisualizationNode::Object3DArrayVisualizationNode(ros::NodeHandle* nh) : nh_(nh)
{
  target_frame_id_ = ros_common_functions::getRosParam<std::string>("~target_frame_id");
  marker_color_ = static_cast<rviz_visual_tools::colors>(ros_common_functions::getRosParam<int>("~color"));
  // marker_color_ = ros_common_functions::getRosParam<std::uint8_t>("~color");
  marker_namespace_ = ros_common_functions::getRosParam<std::string>("~marker_namespace");
  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(target_frame_id_, "vision_markers"));

  object3d_array_sub_ =
      nh->subscribe("object3d_array", 1, &Object3DArrayVisualizationNode::object3DArrayCallback, this);
}

Object3DArrayVisualizationNode::~Object3DArrayVisualizationNode()
{
}

void Object3DArrayVisualizationNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  if (msg->objects.size() > 0)
  {
    if (msg->objects[0].header.frame_id != target_frame_id_)
    {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "[Object3DArrayVisualizationNode] Objects frame id does not match target_frame_id_, transforming objects...");

      // TODO(olek) - transform objects to the target_frame_id_
    }
  }

  updateMarkers(msg);
}

void Object3DArrayVisualizationNode::updateMarkers(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  // NOTE: When msg->objects.size() == 0, we are ok - just deleteAllMarkers() and trigger() are called in such case to
  // clear previous markers

  rviz_visual_tools_->deleteAllMarkers();

  for (const auto& object : msg->objects)
  {
    auto pose = getObjectEigenPose(object);
    rviz_visual_tools_->publishWireframeCuboid(pose, object.bbox.size.x, object.bbox.size.y, object.bbox.size.z,
                                               marker_color_, marker_namespace_);
  }

  rviz_visual_tools_->trigger();
}

Eigen::Isometry3d Object3DArrayVisualizationNode::getObjectEigenPose(const agribot_msgs::Object3D& object) const
{
  Eigen::Isometry3d eigen_pose;
  tf::poseMsgToEigen(object.bbox.center, eigen_pose);
  return eigen_pose;
}

void Object3DArrayVisualizationNode::run() const
{
  ros::spin();
}
