/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/rviz_markers_visualization/Object3DArrayVisualizationNode.h"

Object3DArrayVisualizationNode::Object3DArrayVisualizationNode(ros::NodeHandle* nh)
  : nh_(nh), target_frame_id_(std::nullopt)
{
  marker_color_ = static_cast<rviz_visual_tools::colors>(ros_common_functions::getRosParam<int>("~color"));
  marker_namespace_ = ros_common_functions::getRosParam<std::string>("~marker_namespace");

  object_3d_array_sub_ =
      nh->subscribe("object_3d_array", 1, &Object3DArrayVisualizationNode::object3DArrayCallback, this);
}

void Object3DArrayVisualizationNode::initRvizVisualTools()
{
  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(*target_frame_id_, "vision_markers"));
}

void Object3DArrayVisualizationNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  if (!target_frame_id_)
  {
    if (msg->objects.size() > 0)
    {
      target_frame_id_ = msg->objects[0].header.frame_id;
      initRvizVisualTools();
      ROS_WARN_STREAM("[Object3DArrayVisualizationNode] target_frame_id_ " << *target_frame_id_ << " set");
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "[Object3DArrayVisualizationNode] Message contains no objects, target_frame_id_ not set");
      return;
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
    // Publish wireframe
    auto pose = getObjectEigenPose(object);
    rviz_visual_tools_->publishWireframeCuboid(pose, object.bbox.size.x, object.bbox.size.y, object.bbox.size.z,
                                               marker_color_, marker_namespace_);

    // Publish text in the "upper left" corner of the wireframe
    auto text_pose = pose;
    text_pose.translation()[0] -= object.bbox.size.x / 2.0;
    text_pose.translation()[1] += object.bbox.size.y / 2.0;
    text_pose.translation()[2] += object.bbox.size.z / 2.0;
    auto text = marker_namespace_ + "_" + std::to_string(object.id);
    rviz_visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::TEXTSIZE, false);
  }

  rviz_visual_tools_->trigger(false);
}

Eigen::Isometry3d Object3DArrayVisualizationNode::getObjectEigenPose(const agribot_msgs::Object3D& object) const
{
  Eigen::Isometry3d eigen_pose;
  tf::poseMsgToEigen(object.bbox.center.pose, eigen_pose);
  return eigen_pose;
}

void Object3DArrayVisualizationNode::run() const
{
  ros::spin();
}
