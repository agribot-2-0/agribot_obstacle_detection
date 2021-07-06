/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/util/ObjectConversions.h"

namespace ObjectConversions
{
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

agribot_msgs::ObjectPolygonArray object3dArrayToObjectPolygonArray(const agribot_msgs::Object3DArray& object_array)
{
  agribot_msgs::ObjectPolygonArray msg;

  for (const auto& object : object_array.objects)
  {
    auto polygon = object3dToObjectPolygon(object);
    polygon.type = polygon.OBSTACLE_CLEAR;
    msg.polygons.push_back(polygon);
  }
  return msg;
}

agribot_msgs::ObjectPolygon object3dToObjectPolygon(const agribot_msgs::Object3D& object)
{
  agribot_msgs::ObjectPolygon msg;
  msg.header = object.header;
  msg.type = msg.UNKNOWN;

  // TODO(olek) - Improve object to polygon casting

  // Create polygon of a proper size at 0,0
  geometry_msgs::Point32 polygon_vertex;
  polygon_vertex.z = 0.0;

  // First vertex
  polygon_vertex.x = +object.bbox.size.x / 2;
  polygon_vertex.y = +object.bbox.size.y / 2;
  msg.polygon.points.push_back(polygon_vertex);
  // Seoond vertex
  polygon_vertex.x = +object.bbox.size.x / 2;
  polygon_vertex.y = -object.bbox.size.y / 2;
  msg.polygon.points.push_back(polygon_vertex);
  // Third vertex
  polygon_vertex.x = -object.bbox.size.x / 2;
  polygon_vertex.y = -object.bbox.size.y / 2;
  msg.polygon.points.push_back(polygon_vertex);
  // Fourth vertex
  polygon_vertex.x = -object.bbox.size.x / 2;
  polygon_vertex.y = +object.bbox.size.y / 2;
  msg.polygon.points.push_back(polygon_vertex);

  for (auto& vertex : msg.polygon.points)
  {
    // TODO(olek) - better transform...
    tf::Quaternion q(object.bbox.center.pose.orientation.x, object.bbox.center.pose.orientation.y,
                     object.bbox.center.pose.orientation.z, object.bbox.center.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    float c = cos(yaw);
    float s = sin(yaw);
    float x = vertex.x;
    float y = vertex.y;

    // Rotate polygon around z axis
    vertex.x = x * c - y * s;
    vertex.y = x * s + y * c;

    // Translate polygon to match bbox center
    vertex.x += object.bbox.center.pose.position.x;
    vertex.y += object.bbox.center.pose.position.y;
  }

  return msg;
}

agribot_msgs::ObjectPolygonArray pathToObjectPolygonArray(const nav_msgs::Path& path)
{
  agribot_msgs::ObjectPolygonArray msg;
  for (const auto& pose : path.poses)
  {
    auto polygon = poseStampedToObjectPolygon(pose);
    polygon.type = polygon.PATH_CLEAR;
    msg.polygons.push_back(polygon);
  }
  return msg;
}

agribot_msgs::ObjectPolygon poseStampedToObjectPolygon(const geometry_msgs::PoseStamped& pose)
{
  agribot_msgs::ObjectPolygon msg;
  msg.header = pose.header;
  msg.type = msg.UNKNOWN;

  // TODO(olek) - Improve pose to polygon casting

  // Create polygon of a proper size at 0,0
  geometry_msgs::Point32 polygon_vertex;
  polygon_vertex.z = 0.0;

  // TODO(olek) - Move size to dynam,ic reconfigure
  float x_size = 3.3;
  float y_size = 1.3;

  // First vertex
  polygon_vertex.x = x_size / 2.0;
  polygon_vertex.y = y_size / 2.0;
  msg.polygon.points.push_back(polygon_vertex);
  // Seoond vertex
  polygon_vertex.x = +x_size / 2.0;
  polygon_vertex.y = -y_size / 2.0;
  msg.polygon.points.push_back(polygon_vertex);
  // Third vertex
  polygon_vertex.x = -x_size / 2.0;
  polygon_vertex.y = -y_size / 2.0;
  msg.polygon.points.push_back(polygon_vertex);
  // Fourth vertex
  polygon_vertex.x = -x_size / 2.0;
  polygon_vertex.y = +y_size / 2.0;
  msg.polygon.points.push_back(polygon_vertex);

  for (auto& vertex : msg.polygon.points)
  {
    // TODO(olek) - better transform...
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                     pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    float c = cos(yaw);
    float s = sin(yaw);
    float x = vertex.x;
    float y = vertex.y;

    // Rotate polygon around z axis
    vertex.x = x * c - y * s;
    vertex.y = x * s + y * c;

    // Translate polygon to match bbox center
    vertex.x += pose.pose.position.x;
    vertex.y += pose.pose.position.y;
  }

  return msg;
}

}  // namespace ObjectConversions
