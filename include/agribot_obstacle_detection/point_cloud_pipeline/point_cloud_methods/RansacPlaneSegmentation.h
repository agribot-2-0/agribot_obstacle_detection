/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#pragma GCC diagnostic pop

#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/HelperMethods.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/RansacPlaneSegmentationConfig.h"

template <typename PointT>
void ransacPlaneSegmentation(const typename pcl::PointCloud<PointT>::ConstPtr input_cloud,
                             typename pcl::PointCloud<PointT>::Ptr plane_cloud,
                             typename pcl::PointCloud<PointT>::Ptr objects_cloud,
                             pcl::ModelCoefficients::Ptr coefficients, const RansacPlaneSegmentationConfig& config)
{
  pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
  pcl::SACSegmentation<PointT> seg;

  // Prepare segmentation object
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(config.distance_threshold_);

  if (config.max_iterations_n != 0)
  {
    seg.setMaxIterations(config.max_iterations_n);
  }

  // Do segmentation
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    // Plane not detected
    ROS_WARN_STREAM("[RansacPlaneSegmentation] Could not estimate plane for given dataset.");
    plane_cloud->clear();
    *objects_cloud = *input_cloud;
  }
  else
  {
    // Plane detected - extract inliers and outliers into clouds
    extractInliersOutliers<PointT>(input_cloud, plane_cloud, objects_cloud, inliers);
  }
}
