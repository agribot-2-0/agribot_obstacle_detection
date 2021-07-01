/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/Float64.h>

#include <string>
#include <iomanip>

#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipeline.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipelineNodeConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/VoxelGridFilter.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RadiusOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/StatisticalOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RansacPlaneSegmentation.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/EuclideanClustering.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RegionGrowingClustering.h"

#include "agribot_obstacle_detection/util/ProcessingTimer.h"
#include "agribot_obstacle_detection/util/ObjectConversions.h"

#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"

template <typename PointT, typename DynamicReconfigureT>
class PointCloudPipelineNode
{
private:
  PointCloudPipelineNodeConfig node_config_;
  PointCloudPipeline<PointT> pipeline_;
  agribot_msgs::Object3DArray object_array_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber input_point_cloud_sub_;
  ros::Publisher objects_pub_;
  ros::Publisher ground_point_cloud_pub_;
  ros::Publisher objects_point_cloud_pub_;

  // Processing time publisher
  ros::Publisher processing_time_pub_;

  // TF Buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  typename dynamic_reconfigure::Server<DynamicReconfigureT> dynamic_reconfigure_server_;

  // Logging and debug
  std::string log_tag_;
  ProcessingTimer timer;

public:
  PointCloudPipelineNode(ros::NodeHandle* nh, const std::string& log_tag);
  ~PointCloudPipelineNode() = default;

  void input_point_cloud_cb(const typename pcl::PointCloud<PointT>::ConstPtr& msg);
  void dynamic_reconfigure_callback(const DynamicReconfigureT& dynamic_config, uint32_t level);
  void run() const;
};

template <typename PointT, typename DynamicReconfigureT>
PointCloudPipelineNode<PointT, DynamicReconfigureT>::PointCloudPipelineNode(ros::NodeHandle* nh,
                                                                            const std::string& log_tag)
  : nh_(nh), tf_listener_(tf_buffer_), log_tag_(log_tag)
{
  // Setup dynamic reconfigure server
  typename dynamic_reconfigure::Server<DynamicReconfigureT>::CallbackType dynamic_reconfigure_function =
      boost::bind(&PointCloudPipelineNode::dynamic_reconfigure_callback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_function);

  // Initialize publishers and subscribers
  objects_pub_ = nh->advertise<agribot_msgs::Object3DArray>("objects", 1, true);
  ground_point_cloud_pub_ = nh->advertise<pcl::PointCloud<PointT>>("ground_point_cloud", 1, false);
  objects_point_cloud_pub_ = nh->advertise<pcl::PointCloud<PointT>>("objects_point_cloud", 1, false);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  input_point_cloud_sub_ = nh->subscribe<pcl::PointCloud<PointT>>("input_point_cloud", 1,
                                                                  &PointCloudPipelineNode::input_point_cloud_cb, this);
}

template <typename PointT, typename DynamicReconfigureT>
void PointCloudPipelineNode<PointT, DynamicReconfigureT>::input_point_cloud_cb(
    const typename pcl::PointCloud<PointT>::ConstPtr& msg)
{
  timer.start();
  // TODO(olek) - If config not set (nullopt?) - return and ros_warn_stream

  // Clear list of 3D objects
  object_array_.objects.clear();

  pipeline_.setInputCloud(msg);
  pipeline_.process();
  // auto objects_clusters_ = pipeline_.getObjectsClusters();
  auto objects_clusters_ = pipeline_.getObjectsClusters();

  // Create object for each cluster and push it to Object3DArray
  try
  {
    int id = 0;
    for (const auto& cluster : objects_clusters_)
    {
      auto object =
          ObjectConversions::object3dFromPointCloud<PointT>(cluster, node_config_.objects_target_frame_id_, tf_buffer_);
      object.id = id++;
      object_array_.objects.push_back(object);
    }
  }
  catch (const tf2::LookupException& e)
  {
    ROS_WARN_STREAM(log_tag_ << "Processing pipeline omitted: " << e.what());
    // TODO(olek) - probably we should propagate this error to the arbiter
    return;
  }
  catch (const tf2::ExtrapolationException& e)
  {
    ROS_WARN_STREAM(log_tag_ << "Processing pipeline omitted: " << e.what());
    // TODO(olek) - probably we should propagate this error to the arbiter
    return;
  }

  // Publish Object3DArray
  objects_pub_.publish(object_array_);

  if (node_config_.publish_ground_point_cloud_)
  {
    auto ground_point_cloud = pipeline_.getGroundPointCloud();
    ground_point_cloud_pub_.publish(*ground_point_cloud);
  }

  if (node_config_.publish_objects_point_cloud_)
  {
    auto objects_point_cloud = pipeline_.getObjectsPointCloud();
    objects_point_cloud_pub_.publish(*objects_point_cloud);
  }

  // Stop timer and log results
  timer.stop();
  auto time_ms = timer.get_final_elapsed_time_ms();

  if (node_config_.publish_processing_time_)
  {
    std_msgs::Float64 processing_time_msg;
    processing_time_msg.data = time_ms;
    processing_time_pub_.publish(processing_time_msg);
  }

  if (node_config_.print_processing_time_)
  {
    ROS_INFO_STREAM(log_tag_ << "Full processing took:\t" << std::setprecision(2) << time_ms << " [ms]");
  }
}

template <typename PointT, typename DynamicReconfigureT>
void PointCloudPipelineNode<PointT, DynamicReconfigureT>::dynamic_reconfigure_callback(
    const DynamicReconfigureT& dynamic_config, uint32_t level)
{
  static_cast<void>(level);  // Suppress unused variable warning

  // Get node config
  node_config_.objects_target_frame_id_ = dynamic_config.objects_target_frame_id;
  node_config_.publish_ground_point_cloud_ = dynamic_config.vis_publish_ground_point_cloud;
  node_config_.publish_objects_point_cloud_ = dynamic_config.vis_publish_objects_point_cloud;
  node_config_.publish_processing_time_ = dynamic_config.vis_publish_processing_time;
  node_config_.print_processing_time_ = dynamic_config.vis_print_processing_time;

  // Get pipeline config
  node_config_.pipeline_config_.use_vgf_ = dynamic_config.use_vgf;
  node_config_.pipeline_config_.vgf_config_.resolution_ = dynamic_config.vgf_resolution;
  node_config_.pipeline_config_.vgf_config_.distance_filter_min_x_ = dynamic_config.vgf_min_x;
  node_config_.pipeline_config_.vgf_config_.distance_filter_max_x_ = dynamic_config.vgf_max_x;
  node_config_.pipeline_config_.outlier_removal_method_ =
      static_cast<OutlierRemovalMethod>(dynamic_config.outlier_removal_method);
  node_config_.pipeline_config_.ror_config_.neighbours_n_ = dynamic_config.ror_neighbours_n;
  node_config_.pipeline_config_.ror_config_.radius_ = dynamic_config.ror_radius;
  node_config_.pipeline_config_.sor_config_.neighbours_n_ = dynamic_config.sor_neighbours_n;
  node_config_.pipeline_config_.sor_config_.stddev_mul_ = dynamic_config.sor_stddev_mul;
  node_config_.pipeline_config_.seg_config_.max_iterations_n = dynamic_config.segmentation_iterations_n;
  node_config_.pipeline_config_.seg_config_.distance_threshold_ = dynamic_config.segmentation_distance_threshold;
  node_config_.pipeline_config_.clustering_method_ = static_cast<ClusteringMethod>(dynamic_config.clustering_method);
  node_config_.pipeline_config_.ec_config_.min_cluster_size_ = dynamic_config.clustering_min_cluster_size;
  node_config_.pipeline_config_.ec_config_.max_cluster_size_ = dynamic_config.clustering_max_cluster_size;
  node_config_.pipeline_config_.ec_config_.cluster_tolerance_ = dynamic_config.ec_cluster_tolerance;
  node_config_.pipeline_config_.rgc_config_.min_cluster_size_ = dynamic_config.clustering_min_cluster_size;
  node_config_.pipeline_config_.rgc_config_.max_cluster_size_ = dynamic_config.clustering_max_cluster_size;
  node_config_.pipeline_config_.rgc_config_.normal_estimation_neighbours_n_ =
      dynamic_config.rgc_normal_estimation_neighbours_n;
  node_config_.pipeline_config_.rgc_config_.neighbours_n_ = dynamic_config.rgc_neighbours_n;
  node_config_.pipeline_config_.rgc_config_.smoothness_angle_threshold_ = dynamic_config.rgc_smoothness_angle_threshold;
  node_config_.pipeline_config_.rgc_config_.curvature_threshold_ = dynamic_config.rgc_curvature_threshold;

  pipeline_.setConfig(node_config_.pipeline_config_);
}

template <typename PointT, typename DynamicReconfigureT>
void PointCloudPipelineNode<PointT, DynamicReconfigureT>::run() const
{
  ros::spin();
}

// TODO(olek) - add namespaces!!!
