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
   // nav_msgs::Pathtransform(constnav_msgs::Path&in_msg,conststd::string&target_frame_id,consttf2_ros::Buffer&tf_buffer)

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

}  // namespace ObjectConversions
