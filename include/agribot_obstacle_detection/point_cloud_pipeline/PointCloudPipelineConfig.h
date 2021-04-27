/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <string>

enum class ClusteringMethod : int
{
  // Must match values defined in clustering_method_enum in cfg/LidarPointCloudPipeline.cfg
  euclidean = 0,
  region_growing = 1
};

enum class OutlierRemovalMethod : int
{
  // Must match values defined in outlier_removal_method_enum in cfg/LidarPointCloudPipeline.cfg
  none = 0,
  radius = 1,
  statistical = 2
};

struct PointCloudPipelineConfig
{
  // frame_id to which objects are transformed before publication
  std::string objects_target_frame_id_;

  // VoxelGrid filtering
  bool use_vgf_;
  float vgf_resolution_;
  float vgf_min_x_distance_;
  float vgf_max_x_distance_;

  // Outlier removal filtering
  OutlierRemovalMethod outlier_removal_method_;
  // Radius outlier removal
  int ror_neighbours_n_;
  float ror_radius_;
  // Statistical outlier removal
  int sor_neighbours_n_;
  float sor_stddev_mul_;

  // Ground segmentation
  int segmentation_max_iterations_n_;
  float segmentation_distance_threshold_;

  // Clustering parameters
  ClusteringMethod clustering_method_;
  int clustering_min_cluster_size_;
  int clustering_max_cluster_size_;
  float ec_cluster_tolerance_;
  float rgc_neighbours_n_;
  float rgc_smoothness_angle_threshold_;
  float rgc_curvature_threshold_;
  float rgc_normal_estimation_neighbours_n_;

  // Debug
  bool debug_publish_ground_cloud_;
  bool debug_publish_obstacles_cloud_;
};
