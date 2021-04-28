/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/EuclideanClusteringConfig.h"

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClustering(
    const typename pcl::PointCloud<PointT>::ConstPtr input_cloud, const EuclideanClusteringConfig& config)
{
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  pcl::EuclideanClusterExtraction<PointT> ec;
  std::vector<pcl::PointIndices> cluster_indices;

  ec.setClusterTolerance(config.cluster_tolerance_);
  ec.setMinClusterSize(config.min_cluster_size_);
  ec.setMaxClusterSize(config.max_cluster_size_);
  tree->setInputCloud(input_cloud);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  ec.extract(cluster_indices);

  pcl::ExtractIndices<PointT> extract_indices;
  extract_indices.setInputCloud(input_cloud);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  for (const auto& it : cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

    // TODO(olek) - improve it, as make_shared makes copy of an object
    extract_indices.setIndices(boost::make_shared<pcl::PointIndices>(it));
    extract_indices.filter(*cluster);
    clusters.push_back(cluster);
  }

  return clusters;
}
