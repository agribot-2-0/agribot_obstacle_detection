/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <string>

#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/EuclideanClusteringConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/RadiusOutlierRemovalConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/RansacPlaneSegmentationConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/RegionGrowingClusteringConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/StatisticalOutlierRemovalConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/config/VoxelGridFilterConfig.h"

enum class OutlierRemovalMethod : int
{
  // Must match values defined in outlier_removal_method_enum in cfg/LidarPointCloudPipeline.cfg
  none = 0,
  radius = 1,
  statistical = 2
};

enum class ClusteringMethod : int
{
  // Must match values defined in clustering_method_enum in cfg/LidarPointCloudPipeline.cfg
  euclidean = 0,
  region_growing = 1
};

struct PointCloudPipelineConfig
{
  // VoxelGrid filtering
  bool use_vgf_;
  VoxelGridFilterConfig vgf_config_;

  // Outlier removal filtering
  OutlierRemovalMethod outlier_removal_method_;
  RadiusOutlierRemovalConfig ror_config_;
  StatisticalOutlierRemovalConfig sor_config_;

  // Ground segmentation
  RansacPlaneSegmentationConfig seg_config_;

  // Clustering parameters
  ClusteringMethod clustering_method_;
  EuclideanClusteringConfig ec_config_;
  RegionGrowingClusteringConfig rgc_config_;
};
