/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <tf2_ros/buffer.h>
#include <vision_msgs/BoundingBox3D.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

#include "agribot_msgs/Object3D.h"

template <typename PointT>
geometry_msgs::Point toGeometryMsgsPoint(PointT p)
{
  geometry_msgs::Point point;
  point.x = p.x;
  point.y = p.y;
  point.z = p.z;
  return point;
}

template <typename PointT>
PointT centerPointFromMinMaxPoints(const PointT& min_point, const PointT& max_point)
{
  PointT center_point;
  center_point.x = (max_point.x + min_point.x) / 2.0;
  center_point.y = (max_point.y + min_point.y) / 2.0;
  center_point.z = (max_point.z + min_point.z) / 2.0;
  return center_point;
}

template <typename PointT>
geometry_msgs::Vector3 sizeFromMinMaxPoints(const PointT& min_point, const PointT& max_point)
{
  geometry_msgs::Vector3 size;
  size.x = max_point.x - min_point.x;
  size.y = max_point.y - min_point.y;
  size.z = max_point.z - min_point.z;
  return size;
}

template <typename PointT>
vision_msgs::BoundingBox3D boundingBox3dFromPointCloud(const pcl::PointCloud<PointT>& point_cloud)
{
  PointT min_point, max_point;
  vision_msgs::BoundingBox3D bbox;

  pcl::getMinMax3D(point_cloud, min_point, max_point);
  bbox.center.position = toGeometryMsgsPoint(centerPointFromMinMaxPoints(min_point, max_point));
  // TODO(olek) - calculate bbox orientation
  bbox.center.orientation.w = 1;  // (For now take identity quaternion)
  bbox.size = sizeFromMinMaxPoints(min_point, max_point);

  return bbox;
}

template <typename PointT>
agribot_msgs::Object3D object3dFromPointCloud(typename pcl::PointCloud<PointT>::Ptr point_cloud,
                                              const std::string& target_frame_id, const tf2_ros::Buffer& tf_buffer)
{
  // Note: id and class information are not filled in this function!
  agribot_msgs::Object3D object;

  if (point_cloud->header.frame_id == target_frame_id)
  {
    // Cloud frame is the same as target frame. Just rewrite point_cloud
    pcl::toROSMsg(*point_cloud, object.point_cloud);
    object.bbox = boundingBox3dFromPointCloud(*point_cloud);
  }
  else
  {
    // Transform point_cloud to target frame
    pcl::PointCloud<PointT> temp_cloud;
    auto transform = tf_buffer.lookupTransform(target_frame_id, point_cloud->header.frame_id, ros::Time::now());
    pcl_ros::transformPointCloud(*point_cloud, temp_cloud, transform.transform);
    pcl::toROSMsg(temp_cloud, object.point_cloud);
    object.bbox = boundingBox3dFromPointCloud(temp_cloud);
  }

  object.header = object.point_cloud.header;

  return object;
}
