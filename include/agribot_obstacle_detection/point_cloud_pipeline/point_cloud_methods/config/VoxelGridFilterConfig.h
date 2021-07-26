/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

struct VoxelGridFilterConfig
{
  float resolution_;
  float distance_filter_min_x_;
  float distance_filter_max_x_;
  bool use_passthrough_y_;
  float distance_filter_min_y_;
  float distance_filter_max_y_;
  bool use_passthrough_z_;
  float distance_filter_min_z_;
  float distance_filter_max_z_;
};
