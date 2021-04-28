/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>

#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipelineConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/VoxelGridFilter.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RadiusOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/StatisticalOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RansacPlaneSegmentation.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/EuclideanClustering.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RegionGrowingClustering.h"

template <typename PointT>
class PointCloudPipeline
{
private:
  PointCloudPipelineConfig config_;
  typename pcl::PointCloud<PointT>::ConstPtr input_point_cloud_;
  typename pcl::PointCloud<PointT>::Ptr downsampled_point_cloud_;
  typename pcl::PointCloud<PointT>::Ptr filtered_point_cloud_;
  typename pcl::PointCloud<PointT>::Ptr ground_point_cloud_;
  typename pcl::PointCloud<PointT>::Ptr objects_point_cloud_;
  std::vector<typename pcl::PointCloud<PointT>::Ptr> objects_clusters_;
  pcl::ModelCoefficients::Ptr ground_plane_coefficients_;

public:
  PointCloudPipeline();
  ~PointCloudPipeline() = default;
  void setConfig(const PointCloudPipelineConfig& config);
  void setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& input_point_cloud);
  void process();
  std::vector<typename pcl::PointCloud<PointT>::Ptr>& getObjectsClusters();
  typename pcl::PointCloud<PointT>::Ptr getGroundPointCloud();
  typename pcl::PointCloud<PointT>::Ptr getObjectsPointCloud();
};

template <typename PointT>
PointCloudPipeline<PointT>::PointCloudPipeline()
  : downsampled_point_cloud_(new pcl::PointCloud<PointT>)
  , filtered_point_cloud_(new pcl::PointCloud<PointT>)
  , ground_point_cloud_(new pcl::PointCloud<PointT>)
  , objects_point_cloud_(new pcl::PointCloud<PointT>)
  , ground_plane_coefficients_(new pcl::ModelCoefficients)
{
}

template <typename PointT>
void PointCloudPipeline<PointT>::setConfig(const PointCloudPipelineConfig& config)
{
  config_ = config;
}

template <typename PointT>
void PointCloudPipeline<PointT>::setInputCloud(const typename pcl::PointCloud<PointT>::ConstPtr& input_point_cloud)
{
  input_point_cloud_ = input_point_cloud;
}

template <typename PointT>
void PointCloudPipeline<PointT>::process()
{
  objects_clusters_.clear();

  if (config_.use_vgf_)
  {
    voxelGridFilter<PointT>(input_point_cloud_, downsampled_point_cloud_, config_.vgf_config_);
  }
  else
  {
    // Rewrite pointer if filter is disabled (not the most optimal way, but allows for easier testing)
    // And can't just copy ConstPtr to Ptr
    downsampled_point_cloud_ = boost::make_shared<pcl::PointCloud<PointT>>(*input_point_cloud_);
  }

  switch (config_.outlier_removal_method_)
  {
    case OutlierRemovalMethod::radius:
      radiusOutlierRemoval<PointT>(downsampled_point_cloud_, filtered_point_cloud_, config_.ror_config_);
      break;

    case OutlierRemovalMethod::statistical:
      statisticalOutlierRemoval<PointT>(downsampled_point_cloud_, filtered_point_cloud_, config_.sor_config_);
      break;

    case OutlierRemovalMethod::none:
      // Just rewrite cloud pointer
      filtered_point_cloud_ = downsampled_point_cloud_;
      break;
    default:
      // Just rewrite cloud pointer
      filtered_point_cloud_ = downsampled_point_cloud_;
      break;
  }

  ransacPlaneSegmentation<PointT>(filtered_point_cloud_, ground_point_cloud_, objects_point_cloud_,
                                  ground_plane_coefficients_, config_.seg_config_);

  // Clustering
  switch (config_.clustering_method_)
  {
    case ClusteringMethod::euclidean:
      objects_clusters_ = euclideanClustering<PointT>(objects_point_cloud_, config_.ec_config_);
      break;

    case ClusteringMethod::region_growing:
      objects_clusters_ = regionGrowingClustering<PointT>(objects_point_cloud_, config_.rgc_config_);
      break;
  }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>& PointCloudPipeline<PointT>::getObjectsClusters()
{
  return objects_clusters_;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudPipeline<PointT>::getGroundPointCloud()
{
  return ground_point_cloud_;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudPipeline<PointT>::getObjectsPointCloud()
{
  return objects_point_cloud_;
}
