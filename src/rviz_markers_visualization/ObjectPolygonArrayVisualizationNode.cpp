/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/rviz_markers_visualization/ObjectPolygonArrayVisualizationNode.h"

ObjectPolygonArrayVisualizationNode::ObjectPolygonArrayVisualizationNode(ros::NodeHandle* nh)
  : nh_(nh), target_frame_id_(std::nullopt)
{
  unknown_marker_color_ =
      static_cast<rviz_visual_tools::colors>(ros_common_functions::getRosParam<int>("~unknown_marker_color"));
  obstacle_clear_marker_color_ =
      static_cast<rviz_visual_tools::colors>(ros_common_functions::getRosParam<int>("~obstacle_clear_marker_color_"));
  obstacle_colliding_marker_color_ = static_cast<rviz_visual_tools::colors>(
      ros_common_functions::getRosParam<int>("~obstacle_colliding_marker_color"));
  path_clear_marker_color_ =
      static_cast<rviz_visual_tools::colors>(ros_common_functions::getRosParam<int>("~path_clear_marker_color"));
  path_colliding_marker_color_ =
      static_cast<rviz_visual_tools::colors>(ros_common_functions::getRosParam<int>("~path_colliding_marker_color"));
  marker_namespace_ = ros_common_functions::getRosParam<std::string>("~marker_namespace");

  object_polygon_array_sub_ =
      nh->subscribe("object_polygon_array", 1, &ObjectPolygonArrayVisualizationNode::objectPolygonArrayCallback, this);
}

void ObjectPolygonArrayVisualizationNode::initRvizVisualTools()
{
  rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(*target_frame_id_, "vision_markers"));
}

void ObjectPolygonArrayVisualizationNode::objectPolygonArrayCallback(
    const agribot_msgs::ObjectPolygonArray::ConstPtr& msg)
{
  if (!target_frame_id_)
  {
    if (msg->polygons.size() > 0)
    {
      target_frame_id_ = msg->polygons[0].header.frame_id;
      initRvizVisualTools();
      ROS_WARN_STREAM("[ObjectPolygonArrayVisualizationNode] target_frame_id_ " << *target_frame_id_ << " set");
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "[ObjectPolygonArrayVisualizationNode] Message contains no objects, target_frame_id_ not set");
      return;
    }
  }

  updateMarkers(msg);
}

void ObjectPolygonArrayVisualizationNode::updateMarkers(const agribot_msgs::ObjectPolygonArray::ConstPtr& msg)
{
  // NOTE: When msg->objects.size() == 0, we are ok - just deleteAllMarkers() and trigger() are called in such case to
  // clear previous markers

  rviz_visual_tools_->deleteAllMarkers();

  for (const auto& polygon : msg->polygons)
  {
    if (polygon.type == polygon.UNKNOWN)
    {
      rviz_visual_tools_->publishPolygon(polygon, unknown_marker_color_, rviz_visual_tools::XLARGE,
                                         marker_namespace_ + "_unknown");
    }
    else if (polygon.type == polygon.OBSTACLE_CLEAR)
    {
      rviz_visual_tools_->publishPolygon(polygon, obstacle_clear_marker_color_, rviz_visual_tools::XLARGE,
                                         marker_namespace_ + "_obstacle_clear");
    }
    else if (polygon.type == polygon.OBSTACLE_COLLIDING)
    {
      rviz_visual_tools_->publishPolygon(polygon, obstacle_colliding_marker_color_, rviz_visual_tools::XLARGE,
                                         marker_namespace_ + "_ obstacle_colliding");
    }
    else if (polygon.type == polygon.PATH_CLEAR)
    {
      rviz_visual_tools_->publishPolygon(polygon, path_clear_marker_color_, rviz_visual_tools::XLARGE,
                                         marker_namespace_ + "_path_clear");
    }
    else if (polygon.type == polygon.PATH_COLLIDING)
    {
      rviz_visual_tools_->publishPolygon(polygon, path_colliding_marker_color_, rviz_visual_tools::XLARGE,
                                         marker_namespace_ + "_ path_colliding");
    }
  }

  rviz_visual_tools_->trigger();
}

void ObjectPolygonArrayVisualizationNode::run() const
{
  ros::spin();
}
