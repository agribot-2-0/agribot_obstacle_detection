/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

ObstaclesArbiterNode::ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag)
  : nh_{ nh }, tf_listener_{ tf_buffer_ }, log_tag_{ log_tag }
{
  detection_status_.status = detection_status_.UNKNOWN;

  // Initialize publishers and subscribers
  out_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("out_polygons", 1, true);
  path_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("path_polygons", 1, true);
  objects_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("objects_polygons", 1, true);
  detection_status_pub_ = nh->advertise<agribot_msgs::ObstacleDetectionStatus>("detection_status", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_object_array_sub_ = nh->subscribe("in_objects", 1, &ObstaclesArbiterNode::object3DArrayCallback, this);
  path_sub_ = nh->subscribe("path", 1, &ObstaclesArbiterNode::pathCallback, this);
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
        objects_polygons_ = ObjectConversions::object3dArrayToObjectPolygonArray(
            ObjectConversions::transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        objects_polygons_ = ObjectConversions::object3dArrayToObjectPolygonArray(*msg);
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
        path_polygons_ = ObjectConversions::pathToObjectPolygonArray(
            ObjectConversions::transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        path_polygons_ = ObjectConversions::pathToObjectPolygonArray(*msg);
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
  if (path_polygons_ == std::nullopt)
  {
    // TODO(olek) - change "[ObstaclesArbiterNode] " to use log tag instead
    ROS_WARN_STREAM_THROTTLE(5.0,
                             "[ObstaclesArbiterNode] Missing path info. Skipping calculation of output polygons...");
    return;
  }

  if (objects_polygons_ == std::nullopt)
  {
    ROS_WARN_STREAM_THROTTLE(5.0,
                             "[ObstaclesArbiterNode] Missing objects info. Skipping calculation of output polygons...");
    return;
  }

  // Clear detection status before checking new collisions
  detection_status_.status = detection_status_.NO_OBSTACLE;

  // If there are no polygons in any of the lists, NO_OBSTACLE will be published as for loop will be omitted
  for (auto& path_polygon : path_polygons_->polygons)
  {
    for (auto& object_polygon : objects_polygons_->polygons)
    {
      if (collision_detection_.areColliding(object_polygon, path_polygon))
      {
        detection_status_.status = detection_status_.OBSTACLE_DETECTED;
        publishDetectionStatus();

        // Mark path and objects polygons woth _COLLIDING type
        object_polygon.type = object_polygon.OBSTACLE_COLLIDING;
        path_polygon.type = object_polygon.PATH_COLLIDING;
      }
    }
  }

  // Publish status after collision checking. Mainly to handle NO_OBSTACLE situation
  publishDetectionStatus();

  // Publish output polygons for visualization
  publishOutputPolygons();
}

void ObstaclesArbiterNode::publishOutputPolygons()
{
  // Just rewrite path and obstacles polygons. They should have proper "type" set already
  output_polygons_.polygons.clear();
  output_polygons_.polygons.reserve(objects_polygons_->polygons.size() + path_polygons_->polygons.size());
  output_polygons_.polygons.insert(output_polygons_.polygons.end(), objects_polygons_->polygons.begin(),
                                   objects_polygons_->polygons.end());
  output_polygons_.polygons.insert(output_polygons_.polygons.end(), path_polygons_->polygons.begin(),
                                   path_polygons_->polygons.end());

  out_polygons_pub_.publish(output_polygons_);
}

void ObstaclesArbiterNode::publishDetectionStatus()
{
  detection_status_pub_.publish(detection_status_);
}

void ObstaclesArbiterNode::run() const
{
  ros::spin();
}
