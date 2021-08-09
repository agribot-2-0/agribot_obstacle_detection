/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <nav_msgs/Path.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#pragma GCC diagnostic pop

#include <string>

#include "agribot_obstacle_detection/RowDetectionConfig.h"
#include "agribot_obstacle_detection/row_detection/LineCoefficients.h"
#include "agribot_obstacle_detection/row_detection/PlaneCoefficients.h"
#include "agribot_obstacle_detection/util/Conversions.h"

class RowDetectionNode
{
private:
  agribot_msgs::ObjectPolygonArray row_polygons_;

  double row_path_start_x_;
  double row_path_end_x_;
  double row_polygon_centroid_y_threshold_;
  double row_polygon_area_threshold_;
  double row_plane_cutoff_z_;
  std::string objects_target_frame_id_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber row_object_array_sub_;
  ros::Publisher left_row_path_pub_;
  ros::Publisher right_row_path_pub_;

  // TF Buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  dynamic_reconfigure::Server<agribot_obstacle_detection::RowDetectionConfig> dynamic_reconfigure_server_;

  double eps_angle_;
  double parallel_axis_x_;
  double parallel_axis_y_;
  double parallel_axis_z_;
  double seg_distance_threshold_;
  int seg_max_iterations_;
  int seg_model_type_;
  int seg_method_type_;
  int seg_normal_neighbours_n_;
  double seg_normal_distance_weight_;

  // Logging and debug
  std::string log_tag_;

  rviz_visual_tools::RvizVisualToolsPtr left_row_plane_rviz_tools_;
  rviz_visual_tools::RvizVisualToolsPtr right_row_plane_rviz_tools_;

  void rowObjectsCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
  void computeRowsPaths();
  PlaneCoefficients rowPlaneSegmentation(const sensor_msgs::PointCloud2& pc2);
  LineCoefficients lineFromPlane(const PlaneCoefficients& plane, double cutoff_height) const;
  nav_msgs::Path pathFromLine(const LineCoefficients& line, double start_x, double end_x) const;
  void publishPlaneMarker(const PlaneCoefficients& plane, const std::string& row) const;
  void dynamicReconfigureCallback(const agribot_obstacle_detection::RowDetectionConfig& dynamic_config, uint32_t level);

public:
  RowDetectionNode(ros::NodeHandle* nh, const std::string& log_tag);
  void run() const;
};
