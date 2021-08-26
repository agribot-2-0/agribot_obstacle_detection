/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

struct RansacPlaneSegmentationConfig
{
  int max_iterations_n;
  double distance_threshold_;
  double perpendicular_plane_angle_threshold_;
};
