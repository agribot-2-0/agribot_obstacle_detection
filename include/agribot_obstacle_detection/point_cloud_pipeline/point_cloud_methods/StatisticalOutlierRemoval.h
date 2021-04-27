/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

struct StatisticalOutlierRemovalConfig
{
  int neighbours_n_;
  float stddev_mul_;
};

template <typename PointT>
void statisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                               typename pcl::PointCloud<PointT>::Ptr filtered_cloud,
                               const StatisticalOutlierRemovalConfig& config)
{
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setMeanK(config.neighbours_n_);
  sor.setStddevMulThresh(config.stddev_mul_);
  sor.setInputCloud(input_cloud);
  sor.filter(*filtered_cloud);
}
