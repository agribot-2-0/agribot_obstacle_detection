/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipelineConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/VoxelGridFilter.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RadiusOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/StatisticalOutlierRemoval.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RansacPlaneSegmentation.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/EuclideanClustering.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/point_cloud_methods/RegionGrowingClustering.h"

#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"

template <typename PointT>
class PointCloudPipeline
{
private:
  // PointCloudPipelineConfig config_;


public:
  PointCloudPipeline() = default;
  ~PointCloudPipeline() = default;
};
