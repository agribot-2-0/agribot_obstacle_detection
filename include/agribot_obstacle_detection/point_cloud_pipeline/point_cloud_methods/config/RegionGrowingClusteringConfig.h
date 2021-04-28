/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

struct RegionGrowingClusteringConfig
{
  int normal_estimation_neighbours_n_;
  int neighbours_n_;
  float smoothness_angle_threshold_;
  float curvature_threshold_;
  int min_cluster_size_;
  int max_cluster_size_;
};
