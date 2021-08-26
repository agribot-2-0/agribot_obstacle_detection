/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_hull.h>

#pragma GCC diagnostic ignored "-Wpedantic"
#include <pcl/surface/convex_hull.h>
#pragma GCC diagnostic pop

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

#include <string>

#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"
#include "agribot_msgs/ObjectPolygon.h"
#include "agribot_msgs/ObjectPolygonArray.h"

#include "agribot_obstacle_detection/row_detection/LineCoefficients.h"

namespace Conversions
{
// ==================== //
// Polygon conversions  //
// ==================== //
agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const agribot_msgs::Object3DArray& object_array);
agribot_msgs::ObjectPolygonArray convertToConvexHullPolygonArray(const agribot_msgs::Object3DArray& object_array);
agribot_msgs::ObjectPolygonArray convertToConvexHullPolygonArrayWithRowLimits(
    const agribot_msgs::Object3DArray& object_array, const pcl::PointCloud<pcl::PointXYZ>::Ptr& row_hull_pointcloud);
agribot_msgs::ObjectPolygon convertToObjectPolygon(const agribot_msgs::Object3D& object);
agribot_msgs::ObjectPolygon convertToConvexHullPolygon(const agribot_msgs::Object3D& object);
agribot_msgs::ObjectPolygon convertToConvexHullPolygonWithRowLimits(
    const agribot_msgs::Object3D& object, const pcl::PointCloud<pcl::PointXYZ>::Ptr& row_hull_pointcloud);
agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const nav_msgs::Path& path, double x_size, double y_size);
agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const nav_msgs::Path& path, double min_x_size,
                                                             double min_y_size, double max_x_size, double max_y_size,
                                                             double path_horizon);
agribot_msgs::ObjectPolygon convertToObjectPolygon(const geometry_msgs::PoseStamped& pose, double x_size,
                                                   double y_size);
agribot_msgs::ObjectPolygon getObjectPolygon(double center_x, double center_y, double x_size, double y_size,
                                             double yaw);
LineCoefficients perpendicularLine(const geometry_msgs::PoseStamped& pose);
LineCoefficients lineThroughPoses(const geometry_msgs::PoseStamped& begin_pose,
                                  const geometry_msgs::PoseStamped& end_pose);

// ================ //
// PCL conversions  //
// ================ //
template <typename PointT>
geometry_msgs::Point convertToPointMsg(PointT p);

template <typename PointT>
geometry_msgs::Point32 convertToPoint32Msg(PointT p);

template <typename PointT>
PointT getCenterPoint(const PointT& min_point, const PointT& max_point);

template <typename PointT>
geometry_msgs::Vector3 getSizeVector(const PointT& min_point, const PointT& max_point);

template <typename PointT>
agribot_msgs::BoundingBox3D convertToBoundingBox3D(const pcl::PointCloud<PointT>& point_cloud);

template <typename PointT>
agribot_msgs::Object3D convertToObject3D(typename pcl::PointCloud<PointT>::Ptr point_cloud,
                                         const std::string& target_frame_id, const tf2_ros::Buffer& tf_buffer);

// ================== //
// Eigen conversions  //
// ================== //
Eigen::Vector2f convertToEigenVector2f(const geometry_msgs::Point& point);
Eigen::Vector2f convertToEigenVector2f(const geometry_msgs::Point32& point);
geometry_msgs::Point convertToPointMsg(const Eigen::Vector2f& point_vector);

// ========================== //
// Custom tf transformations  //
// ========================== //
nav_msgs::Path transform(const nav_msgs::Path& in_msg, const std::string& target_frame_id,
                         const tf2_ros::Buffer& tf_buffer);
agribot_msgs::Object3DArray transform(const agribot_msgs::Object3DArray& in_msg, const std::string& target_frame_id,
                                      const tf2_ros::Buffer& tf_buffer);
agribot_msgs::Object3D transform(const agribot_msgs::Object3D& in_msg, const std::string& target_frame_id,
                                 const tf2_ros::Buffer& tf_buffer);
agribot_msgs::BoundingBox3D transform(const agribot_msgs::BoundingBox3D& in_msg,  // NOLINT [build/include_what_you_use]
                                      const std::string& target_frame_id, const tf2_ros::Buffer& tf_buffer);

// ====================== //
// Custom ROS conversions //
// ====================== //
geometry_msgs::Point convertToPointMsg(const geometry_msgs::Point32& p_32);
geometry_msgs::Point32 convertToPoint32Msg(const geometry_msgs::Point& p);

// ================ //
// PCL conversions  //
// ================ //
template <typename PointT>
geometry_msgs::Point convertToPointMsg(PointT p)
{
  geometry_msgs::Point point;
  point.x = p.x;
  point.y = p.y;
  point.z = p.z;
  return point;
}

template <typename PointT>
geometry_msgs::Point32 convertToPoint32Msg(PointT p)
{
  geometry_msgs::Point32 point;
  point.x = p.x;
  point.y = p.y;
  point.z = p.z;
  return point;
}

template <typename PointT>
PointT getCenterPoint(const PointT& min_point, const PointT& max_point)
{
  PointT center_point;
  center_point.x = (max_point.x + min_point.x) / 2.0;
  center_point.y = (max_point.y + min_point.y) / 2.0;
  center_point.z = (max_point.z + min_point.z) / 2.0;
  return center_point;
}

template <typename PointT>
geometry_msgs::Vector3 getSizeVector(const PointT& min_point, const PointT& max_point)
{
  geometry_msgs::Vector3 size;
  size.x = max_point.x - min_point.x;
  size.y = max_point.y - min_point.y;
  size.z = max_point.z - min_point.z;
  return size;
}

template <typename PointT>
agribot_msgs::BoundingBox3D convertToBoundingBox3D(const pcl::PointCloud<PointT>& point_cloud)
{
  PointT min_point, max_point;
  agribot_msgs::BoundingBox3D bbox;

  bbox.center.header = pcl_conversions::fromPCL(point_cloud.header);

  pcl::getMinMax3D(point_cloud, min_point, max_point);
  bbox.center.pose.position = convertToPointMsg(getCenterPoint(min_point, max_point));
  // TODO(olek) - calculate bbox orientation (for now just take identity quaternion)
  bbox.center.pose.orientation.w = 1;
  bbox.size = getSizeVector(min_point, max_point);

  return bbox;
}

template <typename PointT>
agribot_msgs::Object3D convertToObject3D(typename pcl::PointCloud<PointT>::Ptr point_cloud,
                                         const std::string& target_frame_id, const tf2_ros::Buffer& tf_buffer)
{
  // Note: id and class information are not filled in this function!
  agribot_msgs::Object3D object;

  if (point_cloud->header.frame_id == target_frame_id)
  {
    // Cloud frame is the same as target frame. Just rewrite point_cloud
    pcl::toROSMsg(*point_cloud, object.point_cloud);
    object.bbox = convertToBoundingBox3D(*point_cloud);
  }
  else
  {
    // Transform point_cloud to target frame
    pcl::PointCloud<PointT> temp_cloud;
    auto transform = tf_buffer.lookupTransform(target_frame_id, point_cloud->header.frame_id, ros::Time::now());
    pcl_ros::transformPointCloud(*point_cloud, temp_cloud, transform.transform);
    temp_cloud.header.frame_id = target_frame_id;
    pcl::toROSMsg(temp_cloud, object.point_cloud);
    object.bbox = convertToBoundingBox3D(temp_cloud);
  }

  object.header = object.point_cloud.header;

  return object;
}

}  // namespace Conversions
