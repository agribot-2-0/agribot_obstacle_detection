/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/util/Conversions.h"

namespace Conversions
{
// ==================== //
// Polygon conversions  //
// ==================== //
agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const agribot_msgs::Object3DArray& object_array)
{
  agribot_msgs::ObjectPolygonArray msg;

  for (const auto& object : object_array.objects)
  {
    auto polygon = convertToObjectPolygon(object);
    polygon.type = polygon.OBSTACLE_CLEAR;
    msg.polygons.push_back(polygon);
  }
  return msg;
}

agribot_msgs::ObjectPolygonArray convertToConvexHullPolygonArray(const agribot_msgs::Object3DArray& object_array)
{
  agribot_msgs::ObjectPolygonArray msg;

  for (const auto& object : object_array.objects)
  {
    auto polygon = convertToConvexHullPolygon(object);
    polygon.type = polygon.OBSTACLE_CLEAR;
    msg.polygons.push_back(polygon);
  }
  return msg;
}

agribot_msgs::ObjectPolygonArray convertToConvexHullPolygonArrayWithRowLimits(
    const agribot_msgs::Object3DArray& object_array, const pcl::PointCloud<pcl::PointXYZ>::Ptr& row_hull_pointcloud)
{
  agribot_msgs::ObjectPolygonArray msg;

  for (const auto& object : object_array.objects)
  {
    auto polygon = convertToConvexHullPolygonWithRowLimits(object, row_hull_pointcloud);
    polygon.type = polygon.OBSTACLE_CLEAR;

    if (polygon.area > 0)
    {
      msg.polygons.push_back(polygon);
    }
  }
  return msg;
}

agribot_msgs::ObjectPolygon convertToObjectPolygon(const agribot_msgs::Object3D& object)
{
  auto yaw = tf::getYaw(object.bbox.center.pose.orientation);
  auto msg = getObjectPolygon(object.bbox.center.pose.position.x, object.bbox.center.pose.position.y,
                              object.bbox.size.x, object.bbox.size.y, yaw);

  msg.header = object.header;
  msg.type = msg.UNKNOWN;

  return msg;
}

agribot_msgs::ObjectPolygon convertToConvexHullPolygon(const agribot_msgs::Object3D& object)
{
  agribot_msgs::ObjectPolygon msg;

  pcl::PCLPointCloud2 pcl_pointcloud2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl_conversions::toPCL(object.point_cloud, pcl_pointcloud2);
  pcl::fromPCLPointCloud2(pcl_pointcloud2, *pcl_pointcloud);

  for (auto& point : pcl_pointcloud->points)
  {
    point.z = 0.0;
  }

  pcl::ConvexHull<pcl::PointXYZ> convex_hull;
  convex_hull.setInputCloud(pcl_pointcloud);
  convex_hull.setDimension(2);
  convex_hull.setComputeAreaVolume(true);
  convex_hull.reconstruct(*convex_hull_pointcloud);

  for (const auto& point : convex_hull_pointcloud->points)
  {
    auto ros_point32 = Conversions::convertToPoint32Msg<pcl::PointXYZ>(point);
    msg.polygon.points.push_back(ros_point32);
  }

  msg.area = convex_hull.getTotalArea();

  pcl::PointXYZ pcl_centroid_point;
  pcl::computeCentroid<pcl::PointXYZ>(*convex_hull_pointcloud, pcl_centroid_point);
  msg.centroid = Conversions::convertToPoint32Msg<pcl::PointXYZ>(pcl_centroid_point);
  msg.point_cloud = object.point_cloud;

  msg.header = object.header;
  msg.type = msg.UNKNOWN;

  return msg;
}

agribot_msgs::ObjectPolygon convertToConvexHullPolygonWithRowLimits(
    const agribot_msgs::Object3D& object, const pcl::PointCloud<pcl::PointXYZ>::Ptr& row_hull_pointcloud)
{
  agribot_msgs::ObjectPolygon msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr zeroed_pcl_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pcl_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr zeroed_cropped_pcl_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
  std::vector<int> inliers;

  pcl::fromROSMsg(object.point_cloud, *pcl_pointcloud);
  pcl::copyPointCloud(*pcl_pointcloud, *zeroed_pcl_pointcloud);

  for (auto& point : zeroed_pcl_pointcloud->points)
  {
    point.z = 0.0;
  }

  pcl::CropHull<pcl::PointXYZ> crop_hull_filter;
  crop_hull_filter.setHullCloud(row_hull_pointcloud);

  std::vector<pcl::Vertices> polygons;
  pcl::Vertices vertices;
  vertices.vertices.push_back(0);
  vertices.vertices.push_back(1);
  vertices.vertices.push_back(2);
  vertices.vertices.push_back(3);
  polygons.push_back(vertices);
  crop_hull_filter.setHullIndices(polygons);
  crop_hull_filter.setInputCloud(zeroed_pcl_pointcloud);
  crop_hull_filter.setDim(2);
  crop_hull_filter.setCropOutside(true);
  crop_hull_filter.filter(inliers);

  if (inliers.size() < 10)
  {
    // TODO(olek) - it is just ugly workaround used to ignore clouds where all points are cropped as ObjectPolygon still
    // must be returned...
    msg.area = 0.0;
    return msg;
  }

  pcl::copyPointCloud(*pcl_pointcloud, inliers, *cropped_pcl_pointcloud);
  pcl::copyPointCloud(*zeroed_pcl_pointcloud, inliers, *zeroed_cropped_pcl_pointcloud);

  pcl::ConvexHull<pcl::PointXYZ> convex_hull;
  convex_hull.setInputCloud(zeroed_cropped_pcl_pointcloud);
  convex_hull.setDimension(2);
  convex_hull.setComputeAreaVolume(true);
  convex_hull.reconstruct(*convex_hull_pointcloud);

  for (const auto& point : convex_hull_pointcloud->points)
  {
    auto ros_point32 = Conversions::convertToPoint32Msg<pcl::PointXYZ>(point);
    msg.polygon.points.push_back(ros_point32);
  }

  msg.area = convex_hull.getTotalArea();

  pcl::PointXYZ pcl_centroid_point;
  pcl::computeCentroid<pcl::PointXYZ>(*convex_hull_pointcloud, pcl_centroid_point);
  msg.centroid = Conversions::convertToPoint32Msg<pcl::PointXYZ>(pcl_centroid_point);

  pcl::toROSMsg(*cropped_pcl_pointcloud, msg.point_cloud);

  msg.header = object.header;
  msg.type = msg.UNKNOWN;

  return msg;
}

agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const nav_msgs::Path& path, double x_size, double y_size)
{
  agribot_msgs::ObjectPolygonArray msg;
  for (const auto& pose : path.poses)
  {
    auto polygon = convertToObjectPolygon(pose, x_size, y_size);
    polygon.type = polygon.PATH_CLEAR;
    msg.polygons.push_back(polygon);
  }
  return msg;
}

agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const nav_msgs::Path& path, double min_x_size,
                                                             double min_y_size, double max_x_size, double max_y_size,
                                                             double path_horizon)
{
  agribot_msgs::ObjectPolygonArray msg;
  for (const auto& pose : path.poses)
  {
    auto distance = std::hypot(pose.pose.position.x, pose.pose.position.y);

    if (distance < path_horizon)
    {
      auto size_coefficient = 1.0 - (distance / path_horizon);

      auto x_size_diff = max_x_size - min_x_size;
      auto y_size_diff = max_y_size - min_y_size;

      auto x_size = min_x_size + size_coefficient * x_size_diff;
      auto y_size = min_y_size + size_coefficient * y_size_diff;

      auto polygon = convertToObjectPolygon(pose, x_size, y_size);
      polygon.type = polygon.PATH_CLEAR;
      msg.polygons.push_back(polygon);
    }
  }
  return msg;
}

agribot_msgs::ObjectPolygon convertToObjectPolygon(const geometry_msgs::PoseStamped& pose, double x_size, double y_size)
{
  auto yaw = tf::getYaw(pose.pose.orientation);
  auto msg = getObjectPolygon(pose.pose.position.x, pose.pose.position.y, x_size, y_size, yaw);

  msg.header = pose.header;
  msg.type = msg.UNKNOWN;
  msg.area = x_size * y_size;
  msg.centroid = Conversions::convertToPoint32Msg(pose.pose.position);

  return msg;
}

agribot_msgs::ObjectPolygon getObjectPolygon(double center_x, double center_y, double x_size, double y_size, double yaw)
{
  agribot_msgs::ObjectPolygon msg;

  geometry_msgs::Point32 polygon_vertex;
  auto c = cos(yaw);
  auto s = sin(yaw);
  double half_x_size = x_size / 2.0;
  double half_y_size = y_size / 2.0;
  double x, y;

  // First vertex
  x = half_x_size;
  y = half_y_size;
  polygon_vertex.x = c * x - s * y + center_x;
  polygon_vertex.y = s * x + c * y + center_y;
  polygon_vertex.z = 0.0;
  msg.polygon.points.push_back(polygon_vertex);

  // Seoond vertex
  x = half_x_size;
  y = -half_y_size;
  polygon_vertex.x = c * x - s * y + center_x;
  polygon_vertex.y = s * x + c * y + center_y;
  polygon_vertex.z = 0.0;
  msg.polygon.points.push_back(polygon_vertex);

  // Third vertex
  x = -half_x_size;
  y = -half_y_size;
  polygon_vertex.x = c * x - s * y + center_x;
  polygon_vertex.y = s * x + c * y + center_y;
  polygon_vertex.z = 0.0;
  msg.polygon.points.push_back(polygon_vertex);

  // Fourth vertex
  x = -half_x_size;
  y = half_y_size;
  polygon_vertex.x = c * x - s * y + center_x;
  polygon_vertex.y = s * x + c * y + center_y;
  polygon_vertex.z = 0.0;
  msg.polygon.points.push_back(polygon_vertex);

  return msg;
}

LineCoefficients perpendicularLine(const geometry_msgs::PoseStamped& pose)
{
  auto yaw = tf::getYaw(pose.pose.orientation);
  double slope;

  if (yaw == 0.0)
  {
    slope = 0.0;
  }
  else
  {
    slope = std::tan(yaw);
  }

  // Point-slope equation for perpendicular line
  // y = ax + b, with a=-1/m and b=1/m*x1 + y1
  // where:
  // m - slope
  // (x1, y1) - point
  // (a, b) - line coefficients
  auto a = -1.0 / slope;
  auto b = 1.0 / slope * pose.pose.position.x + pose.pose.position.y;

  return LineCoefficients{ a, b };
}

LineCoefficients lineThroughPoses(const geometry_msgs::PoseStamped& begin_pose,
                                  const geometry_msgs::PoseStamped& end_pose)
{
  auto slope =
      (end_pose.pose.position.y - begin_pose.pose.position.y) / (end_pose.pose.position.x - begin_pose.pose.position.x);

  return { slope, -slope * begin_pose.pose.position.x + begin_pose.pose.position.y };
}

// ================== //
// Eigen conversions  //
// ================== //
Eigen::Vector2f convertToEigenVector2f(const geometry_msgs::Point& point)
{
  return Eigen::Vector2f(point.x, point.y);
}

Eigen::Vector2f convertToEigenVector2f(const geometry_msgs::Point32& point)
{
  return Eigen::Vector2f(point.x, point.y);
}

geometry_msgs::Point convertToPointMsg(const Eigen::Vector2f& point_vector)
{
  geometry_msgs::Point point;
  point.x = point_vector[0];
  point.y = point_vector[1];
  return point;
}

// ========================== //
// Custom tf transformations  //
// ========================== //
nav_msgs::Path transform(const nav_msgs::Path& in_msg, const std::string& target_frame_id,
                         const tf2_ros::Buffer& tf_buffer)
{
  nav_msgs::Path out_msg;

  // TODO(olek) - use lookupTransform instead of transform
  for (const auto& pose : in_msg.poses)
  {
    out_msg.poses.push_back(tf_buffer.transform(pose, target_frame_id));
  }

  out_msg.header = in_msg.header;
  out_msg.header.frame_id = target_frame_id;

  return out_msg;
}  // namespace

agribot_msgs::Object3DArray transform(const agribot_msgs::Object3DArray& in_msg, const std::string& target_frame_id,
                                      const tf2_ros::Buffer& tf_buffer)
{
  agribot_msgs::Object3DArray out_msg;

  // TODO(olek) - use lookupTransform instead of transform
  for (const auto& object : in_msg.objects)
  {
    out_msg.objects.push_back(transform(object, target_frame_id, tf_buffer));
  }

  return out_msg;
}

agribot_msgs::Object3D transform(const agribot_msgs::Object3D& in_msg, const std::string& target_frame_id,
                                 const tf2_ros::Buffer& tf_buffer)
{
  agribot_msgs::Object3D out_msg;

  // TODO(olek) - use lookupTransform instead of transform
  out_msg.header = in_msg.header;
  out_msg.header.frame_id = target_frame_id;
  out_msg.id = in_msg.id;
  out_msg.classes = in_msg.classes;
  out_msg.point_cloud = tf_buffer.transform(in_msg.point_cloud, target_frame_id);
  out_msg.bbox = transform(in_msg.bbox, target_frame_id, tf_buffer);
  return out_msg;
}

agribot_msgs::BoundingBox3D transform(const agribot_msgs::BoundingBox3D& in_msg, const std::string& target_frame_id,
                                      const tf2_ros::Buffer& tf_buffer)
{
  agribot_msgs::BoundingBox3D out_msg;

  // TODO(olek) - use lookupTransform instead of transform
  out_msg.center = tf_buffer.transform(in_msg.center, target_frame_id);
  out_msg.center.header.frame_id = target_frame_id;
  out_msg.size = in_msg.size;

  return out_msg;
}

// ====================== //
// Custom ROS conversions //
// ====================== //
geometry_msgs::Point convertToPointMsg(const geometry_msgs::Point32& p_32)
{
  geometry_msgs::Point p;
  p.x = p_32.x;
  p.y = p_32.y;
  p.z = p_32.z;
  return p;
}

geometry_msgs::Point32 convertToPoint32Msg(const geometry_msgs::Point& p)
{
  geometry_msgs::Point32 p_32;
  p_32.x = p.x;
  p_32.y = p.y;
  p_32.z = p.z;
  return p_32;
}

}  // namespace Conversions
