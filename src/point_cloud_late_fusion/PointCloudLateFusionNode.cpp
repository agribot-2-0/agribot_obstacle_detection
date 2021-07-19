/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/point_cloud_late_fusion/PointCloudLateFusionNode.h"

PointCloudLateFusionNode::PointCloudLateFusionNode(ros::NodeHandle* nh, const std::string& log_tag)
  : nh_(nh), tf_listener_(tf_buffer_), log_tag_(log_tag)
{
  // Setup dynamic reconfigure server
  typename dynamic_reconfigure::Server<agribot_obstacle_detection::PointCloudLateFusionConfig>::CallbackType
      dynamic_reconfigure_function = boost::bind(&PointCloudLateFusionNode::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_function);

  // Initialize publishers and subscribers
  out_object_array_pub_ = nh->advertise<agribot_msgs::Object3DArray>("out_objects", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_1_object_array_sub_ = nh->subscribe<agribot_msgs::Object3DArray>(
      "in_1_objects", 1, boost::bind(&PointCloudLateFusionNode::objectArrayCallback, this, _1, 1));
  in_2_object_array_sub_ = nh->subscribe<agribot_msgs::Object3DArray>(
      "in_2_objects", 1, boost::bind(&PointCloudLateFusionNode::objectArrayCallback, this, _1, 2));
}

void PointCloudLateFusionNode::objectArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg, int id)
{
  switch (id)
  {
    case 1:
      in_1_object_array_ = *msg;
      break;
    case 2:
      in_2_object_array_ = *msg;
      break;
  }

  // Check if we have all necessary data
  switch (fusion_method_)
  {
    case LateFusionMethod::rewrite_lidar:
      if (in_1_object_array_ == std::nullopt)
      {
        ROS_WARN_STREAM(
            log_tag_ << "Missing object array from one of the inputs. Skipping output array publication...");
        return;
      }
      break;
    case LateFusionMethod::rewrite_stereo_camera:
      if (in_2_object_array_ == std::nullopt)
      {
        ROS_WARN_STREAM(
            log_tag_ << "Missing object array from one of the inputs. Skipping output array publication...");
        return;
      }
      break;
    default:
      if (in_1_object_array_ == std::nullopt || in_2_object_array_ == std::nullopt)
      {
        ROS_WARN_STREAM(
            log_tag_ << "Missing object array from one of the inputs. Skipping output array publication...");
        return;
      }
      break;
  }

  publishOutObjectArrary(objectArrayFusion());
}

agribot_msgs::Object3DArray PointCloudLateFusionNode::objectArrayFusion()
{
  // For now just copy from both vectors
  switch (fusion_method_)
  {
    case LateFusionMethod::rewrite_lidar:
      return *in_1_object_array_;

    case LateFusionMethod::rewrite_stereo_camera:
      return *in_2_object_array_;

    case LateFusionMethod::intersection_over_union:
      return intersectionOverUnionFusion();

    case LateFusionMethod::rewrite_both:
    default:
      agribot_msgs::Object3DArray out_object_array_;
      out_object_array_.objects.reserve(in_1_object_array_->objects.size() + in_2_object_array_->objects.size());
      out_object_array_.objects.insert(out_object_array_.objects.end(), in_1_object_array_->objects.begin(),
                                       in_1_object_array_->objects.end());
      out_object_array_.objects.insert(out_object_array_.objects.end(), in_2_object_array_->objects.begin(),
                                       in_2_object_array_->objects.end());
      return out_object_array_;
  }
}

agribot_msgs::Object3DArray PointCloudLateFusionNode::intersectionOverUnionFusion()
{
  agribot_msgs::Object3DArray out_object_array_;

  // We trust lidar more, so we rewrite lidar's object if iou_threshold_ is reached
  for (const auto& lidar_object : in_1_object_array_->objects)
  {
    for (const auto& stereo_camera_object : in_2_object_array_->objects)
    {
      if (intersectionOverUnion(lidar_object, stereo_camera_object) >= iou_threshold_)
      {
        out_object_array_.objects.push_back(lidar_object);
      }
    }
  }

  return out_object_array_;
}

void PointCloudLateFusionNode::publishOutObjectArrary(const agribot_msgs::Object3DArray& msg)
{
  out_object_array_pub_.publish(msg);
}

void PointCloudLateFusionNode::dynamicReconfigureCallback(
    const agribot_obstacle_detection::PointCloudLateFusionConfig& dynamic_config, uint32_t level)
{
  static_cast<void>(level);  // Suppress unused variable warning

  // Get node config
  fusion_method_ = static_cast<LateFusionMethod>(dynamic_config.fusion_method);
  iou_threshold_ = dynamic_config.iou_threshold;
  objects_target_frame_id_ = dynamic_config.objects_target_frame_id;
  publish_processing_time_ = dynamic_config.debug_publish_processing_time;
  print_processing_time_ = dynamic_config.debug_print_processing_time;
}

void PointCloudLateFusionNode::run() const
{
  ros::spin();
}
