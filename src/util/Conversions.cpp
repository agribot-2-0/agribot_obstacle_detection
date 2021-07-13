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

agribot_msgs::ObjectPolygon convertToObjectPolygon(const agribot_msgs::Object3D& object)
{
  agribot_msgs::ObjectPolygon msg;
  msg.header = object.header;
  msg.type = msg.UNKNOWN;

  float half_x_size = object.bbox.size.x / 2.0;
  float half_y_size = object.bbox.size.y / 2.0;

  // Create polygon of a proper size at 0,0
  geometry_msgs::Point32 polygon_vertex;
  polygon_vertex.z = 0.0;

  // First vertex
  polygon_vertex.x = +half_x_size;
  polygon_vertex.y = +half_y_size;
  msg.polygon.points.push_back(polygon_vertex);
  // Seoond vertex
  polygon_vertex.x = +half_x_size;
  polygon_vertex.y = -half_y_size;
  msg.polygon.points.push_back(polygon_vertex);
  // Third vertex
  polygon_vertex.x = -half_x_size;
  polygon_vertex.y = -half_y_size;
  msg.polygon.points.push_back(polygon_vertex);
  // Fourth vertex
  polygon_vertex.x = -half_x_size;
  polygon_vertex.y = +half_y_size;
  msg.polygon.points.push_back(polygon_vertex);

  for (auto& vertex : msg.polygon.points)
  {
    auto yaw = tf::getYaw(object.bbox.center.pose.orientation);
    auto c = cos(yaw);
    auto s = sin(yaw);
    auto x = vertex.x;
    auto y = vertex.y;

    // Rotate polygon around z axis
    vertex.x = x * c - y * s;
    vertex.y = x * s + y * c;

    // Translate polygon to match bbox center
    vertex.x += object.bbox.center.pose.position.x;
    vertex.y += object.bbox.center.pose.position.y;
  }

  return msg;
}

agribot_msgs::ObjectPolygonArray convertToObjectPolygonArray(const nav_msgs::Path& path, float x_size, float y_size)
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

agribot_msgs::ObjectPolygon convertToObjectPolygon(const geometry_msgs::PoseStamped& pose, float x_size, float y_size)
{
  agribot_msgs::ObjectPolygon msg;
  msg.header = pose.header;
  msg.type = msg.UNKNOWN;

  float half_x_size = x_size / 2.0;
  float half_y_size = y_size / 2.0;

  // Create polygon of a proper size at 0,0
  geometry_msgs::Point32 polygon_vertex;
  polygon_vertex.z = 0.0;

  // First vertex
  polygon_vertex.x = half_x_size;
  polygon_vertex.y = half_y_size;
  msg.polygon.points.push_back(polygon_vertex);
  // Seoond vertex
  polygon_vertex.x = +half_x_size;
  polygon_vertex.y = -half_y_size;
  msg.polygon.points.push_back(polygon_vertex);
  // Third vertex
  polygon_vertex.x = -half_x_size;
  polygon_vertex.y = -half_y_size;
  msg.polygon.points.push_back(polygon_vertex);
  // Fourth vertex
  polygon_vertex.x = -half_x_size;
  polygon_vertex.y = +half_y_size;
  msg.polygon.points.push_back(polygon_vertex);

  for (auto& vertex : msg.polygon.points)
  {
    auto yaw = tf::getYaw(pose.pose.orientation);
    auto c = cos(yaw);
    auto s = sin(yaw);
    auto x = vertex.x;
    auto y = vertex.y;

    // Rotate polygon around z axis
    vertex.x = x * c - y * s;
    vertex.y = x * s + y * c;

    // Translate polygon to match bbox center
    vertex.x += pose.pose.position.x;
    vertex.y += pose.pose.position.y;
  }

  return msg;
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
  out_msg.size = in_msg.size;

  return out_msg;
}

}  // namespace Conversions
