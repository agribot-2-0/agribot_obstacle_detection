/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

enum class LateFusionMethod : int
{
  // Must match values defined in outlier_removal_method_enum in cfg/LidarPointCloudPipeline.cfg
  rewrite_both = 0,
  rewrite_lidar = 1,
  rewrite_stereo_camera = 2,
  intersection_over_union = 3
};
