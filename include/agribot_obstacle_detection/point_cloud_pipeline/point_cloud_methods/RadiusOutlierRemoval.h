/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/RadiusOutlierRemovalConfig.h"

template <typename PointT>
void radiusOutlierRemoval(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                          typename pcl::PointCloud<PointT>::Ptr filtered_cloud,
                          const RadiusOutlierRemovalConfig& config)
{
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setMinNeighborsInRadius(config.neighbours_n_);
  ror.setRadiusSearch(config.radius_);
  ror.setInputCloud(input_cloud);
  ror.filter(*filtered_cloud);
}
