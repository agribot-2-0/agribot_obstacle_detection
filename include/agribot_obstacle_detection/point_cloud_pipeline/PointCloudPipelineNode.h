/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipelineConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/VoxelGridFilter.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RadiusOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/StatisticalOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RansacPlaneSegmentation.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/EuclideanClustering.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RegionGrowingClustering.h"

#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"

template <typename PointT, typename DynamicReconfigureT>
class PointCloudPipelineNode
{
private:
  PointCloudPipelineConfig config_;

  // Dynamic reconfigure
  typename dynamic_reconfigure::Server<DynamicReconfigureT> dynamic_reconfigure_server_;
  // dynamic_reconfigure::Server<DynamicReconfigureT>::CallbackType dynamic_reconfigure_function;

  agribot_msgs::Object3DArray objects_;

  ros::NodeHandle* nh_;

public:
  explicit PointCloudPipelineNode(ros::NodeHandle* nh);
  ~PointCloudPipelineNode() = default;

  void dynamic_reconfigure_callback(const DynamicReconfigureT& dynamic_config, uint32_t level);
  void run();
};

template <typename PointT, typename DynamicReconfigureT>
PointCloudPipelineNode<PointT, DynamicReconfigureT>::PointCloudPipelineNode(ros::NodeHandle* nh) : nh_(nh)
{
  // Setup dynamic reconfigure server
  typename dynamic_reconfigure::Server<DynamicReconfigureT>::CallbackType dynamic_reconfigure_function =
      boost::bind(&PointCloudPipelineNode::dynamic_reconfigure_callback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_function);
}

template <typename PointT, typename DynamicReconfigureT>
void PointCloudPipelineNode<PointT, DynamicReconfigureT>::run()
{
  ros::spin();
}

template <typename PointT, typename DynamicReconfigureT>
void PointCloudPipelineNode<PointT, DynamicReconfigureT>::dynamic_reconfigure_callback(
    const DynamicReconfigureT& dynamic_config, uint32_t level)
{
  ROS_ERROR_STREAM("[Dynamic reconfigure callback] vgf_resolution = " << dynamic_config.vgf_resolution);
}

// TODO - add namespaces!!!
