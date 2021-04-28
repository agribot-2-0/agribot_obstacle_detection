/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipelineConfig.h"

struct PointCloudPipelineNodeConfig
{
  PointCloudPipelineConfig pipeline_config_;

  // frame_id to which objects are transformed before publication
  std::string objects_target_frame_id_;

  // Visualization and debug
  bool publish_ground_point_cloud_;
  bool publish_objects_point_cloud_;
  bool publish_processing_time_;
  bool print_processing_time_;
};
