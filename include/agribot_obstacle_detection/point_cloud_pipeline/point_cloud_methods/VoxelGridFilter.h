/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

struct VoxelGridFilterConfig
{
  float filter_resolution_;
  float distance_filter_min_x_;
  float distance_filter_max_x_;
};

template <typename PointT>
void voxelGridFilter(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                     typename pcl::PointCloud<PointT>::Ptr filtered_cloud, const VoxelGridFilterConfig& config)
{
  pcl::VoxelGrid<PointT> vg;

  // Config filter
  vg.setFilterFieldName("x");
  vg.setFilterLimits(config.distance_filter_min_x_, config.distance_filter_max_x_);
  vg.setLeafSize(config.filter_resolution_, config.filter_resolution_, config.filter_resolution_);

  // Set input
  vg.setInputCloud(input_cloud);

  // Filtering
  vg.filter(*filtered_cloud);
}
