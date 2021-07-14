/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>

template <typename PointT>
void extractInliersOutliers(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                            typename pcl::PointCloud<PointT>::Ptr inliers_cloud,
                            typename pcl::PointCloud<PointT>::Ptr outliers_cloud, pcl::PointIndices::Ptr inliers)
{
  pcl::ExtractIndices<PointT> extract;

  // Set extractor input
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);

  // Extract inliers cloud
  extract.setNegative(false);
  extract.filter(*inliers_cloud);

  // Extract outliers cloud (negative of inliers)
  extract.setNegative(true);
  extract.filter(*outliers_cloud);
}
