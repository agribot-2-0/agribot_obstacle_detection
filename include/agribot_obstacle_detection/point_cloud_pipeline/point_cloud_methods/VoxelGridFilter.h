/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/VoxelGridFilterConfig.h"

template <typename PointT>
void voxelGridFilter(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                     typename pcl::PointCloud<PointT>::Ptr filtered_cloud, const VoxelGridFilterConfig& config)
{
  pcl::VoxelGrid<PointT> vg;

  typename pcl::PointCloud<PointT>::Ptr y_filtered_cloud{ new pcl::PointCloud<PointT> };
  typename pcl::PointCloud<PointT>::Ptr z_filtered_cloud{ new pcl::PointCloud<PointT> };

  if (config.use_passthrough_y_)
  {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(config.distance_filter_min_y_, config.distance_filter_max_y_);
    pass.filter(*y_filtered_cloud);
  }
  else
  {
    *y_filtered_cloud = *input_cloud;
  }

  if (config.use_passthrough_z_)
  {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(y_filtered_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(config.distance_filter_min_z_, config.distance_filter_max_z_);
    pass.filter(*z_filtered_cloud);
  }
  else
  {
    *z_filtered_cloud = *y_filtered_cloud;
  }

  // Config filter
  vg.setFilterFieldName("x");
  vg.setFilterLimits(config.distance_filter_min_x_, config.distance_filter_max_x_);
  vg.setLeafSize(config.resolution_, config.resolution_, config.resolution_);

  // Set input
  vg.setInputCloud(z_filtered_cloud);

  // Filtering
  vg.filter(*filtered_cloud);
}
