/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <vector>

#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/RegionGrowingClusteringConfig.h"

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> regionGrowingClustering(
    const typename pcl::PointCloud<PointT>::ConstPtr input_cloud, const RegionGrowingClusteringConfig& config)
{
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

  // Storing intermediate results
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  // Prepare search method
  tree->setInputCloud(input_cloud);

  // Prepare normal estimator
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setInputCloud(input_cloud);
  normal_estimation.setKSearch(config.normal_estimation_neighbours_n_);
  // normal_estimation.setRadiusSearch(0.03);
  normal_estimation.compute(*normals);

  pcl::RegionGrowing<PointT, pcl::Normal> rgc;
  rgc.setMinClusterSize(config.min_cluster_size_);
  rgc.setMaxClusterSize(config.max_cluster_size_);
  rgc.setSearchMethod(tree);
  rgc.setNumberOfNeighbours(config.neighbours_n_);
  rgc.setInputCloud(input_cloud);
  rgc.setInputNormals(normals);
  rgc.setSmoothnessThreshold(config.smoothness_angle_threshold_ / 180.0 * M_PI);
  rgc.setCurvatureThreshold(config.curvature_threshold_);

  std::vector<pcl::PointIndices> cluster_indices;
  rgc.extract(cluster_indices);

  pcl::ExtractIndices<PointT> extract_indices;
  extract_indices.setInputCloud(input_cloud);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  for (const auto& it : cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    extract_indices.setIndices(boost::make_shared<pcl::PointIndices>(it));
    extract_indices.filter(*cluster);
    clusters.push_back(cluster);
  }

  return clusters;
}
