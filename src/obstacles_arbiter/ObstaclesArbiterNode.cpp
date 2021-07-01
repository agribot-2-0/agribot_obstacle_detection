/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

ObstaclesArbiterNode::ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag)
  : nh_(nh), tf_listener_(tf_buffer_), log_tag_(log_tag)
{
  // Initialize publishers and subscribers
  out_polygons_pub_ = nh->advertise<agribot_msgs::Object3DArray>("out_polygons", 1, true);
  path_polygons_pub_ = nh->advertise<agribot_msgs::Object3DArray>("path_polygons", 1, true);
  objects_polygons_pub_ = nh->advertise<agribot_msgs::Object3DArray>("objects_polygons", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_object_array_sub_ = nh->subscribe("in_objects", 1, &ObstaclesArbiterNode::object3DArrayCallback, this);
  path_sub_ = nh->subscribe("path", 1, &ObstaclesArbiterNode::pathCallback, this);
}

agribot_msgs::ObjectPolygonArray
ObstaclesArbiterNode::object3dArrayToObjectPolygonArray(const agribot_msgs::Object3DArray& object_array)
{
  agribot_msgs::ObjectPolygonArray msg;
  // TODO(olek)
  return msg;
}

agribot_msgs::ObjectPolygonArray ObstaclesArbiterNode::pathToObjectPolygonArray(const nav_msgs::Path& path)
{
  agribot_msgs::ObjectPolygonArray msg;
  // TODO(olek)
  return msg;
}

void ObstaclesArbiterNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  try
  {
    if (msg->objects.size() == 0)
    {
      objects_polygons_->polygons.clear();
    }
    else
    {
      if (msg->objects[0].header.frame_id != objects_target_frame_id_)
      {
        objects_polygons_ = object3dArrayToObjectPolygonArray(transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        objects_polygons_ = object3dArrayToObjectPolygonArray(*msg);
      }
    }

    calculateOutputPolygons();
    publishOutputPolygons();
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("[ObstaclesArbiterNode] " << e.what());
  }
}

void ObstaclesArbiterNode::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  try
  {
    if (msg->poses.size() == 0)
    {
      path_polygons_->polygons.clear();
    }
    else
    {
      if (msg->header.frame_id != objects_target_frame_id_)
      {
        path_polygons_ = pathToObjectPolygonArray(transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        path_polygons_ = pathToObjectPolygonArray(*msg);
      }
    }

    calculateOutputPolygons();
    publishOutputPolygons();
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM("[ObstaclesArbiterNode] " << e.what());
  }
}

void ObstaclesArbiterNode::calculateOutputPolygons()
{
  if (path_polygons_ == std::nullopt || objects_polygons_ == std::nullopt)
  {
    return;
  }

  if (path_polygons_->polygons.size() == 0 || objects_polygons_->polygons.size() == 0)
  {
    output_polygons_->polygons.clear();
    return;
  }

  // TODO(olek)
  // output_polygons_
}

void ObstaclesArbiterNode::publishOutputPolygons()
{
  if (output_polygons_ != std::nullopt)
  {
    out_polygons_pub_.publish(*output_polygons_);
  }
}

void ObstaclesArbiterNode::run() const
{
  ros::spin();
}
