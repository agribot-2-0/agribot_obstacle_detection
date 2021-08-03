/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#pragma GCC diagnostic pop

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "agribot_common/CommonFunctions/ros_common_functions.h"
#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"
#include "agribot_msgs/ObjectPolygon.h"
#include "agribot_msgs/ObjectPolygonArray.h"
#include "agribot_msgs/ObstacleDetectionStatus.h"

#include "agribot_obstacle_detection/ObstaclesArbiterConfig.h"
#include "agribot_obstacle_detection/obstacles_arbiter/collision_detection/IPolygonCollisionDetection.h"
#include "agribot_obstacle_detection/util/Conversions.h"
#include "agribot_obstacle_detection/util/ProcessingTimer.h"

class ObstaclesArbiterNode
{
private:
  double robot_safety_zone_length_;
  double robot_safety_zone_width_;
  double min_path_safety_zone_length_;
  double min_path_safety_zone_width_;
  double max_path_safety_zone_length_;
  double max_path_safety_zone_width_;
  double path_safety_zone_horizon_;
  double row_path_start_x_;
  double row_path_end_x_;
  double row_polygon_centroid_y_threshold_;
  double row_polygon_area_threshold_;
  double row_plane_cutoff_z_;
  std::string objects_target_frame_id_;
  bool publish_processing_time_;
  bool print_processing_time_;

  std::optional<agribot_msgs::ObjectPolygonArray> path_polygons_;
  std::optional<agribot_msgs::ObjectPolygonArray> objects_polygons_;
  std::optional<agribot_msgs::ObjectPolygonArray> row_polygons_;
  agribot_msgs::ObjectPolygonArray output_polygons_;
  agribot_msgs::ObjectPolygon robot_polygon_;
  agribot_msgs::ObstacleDetectionStatus detection_status_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber in_object_array_sub_;
  ros::Subscriber row_object_array_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher out_polygons_pub_;
  ros::Publisher path_polygons_pub_;
  ros::Publisher objects_polygons_pub_;
  ros::Publisher detection_status_pub_;
  ros::Publisher left_row_path_pub_;
  ros::Publisher right_row_path_pub_;

  // Processing time publisher
  ros::Publisher processing_time_pub_;

  // TF Buffer and listener
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  dynamic_reconfigure::Server<agribot_obstacle_detection::ObstaclesArbiterConfig> dynamic_reconfigure_server_;

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
  ProcessingTimer timer;

  std::unique_ptr<IPolygonCollisionDetection> collision_detection_;

  rviz_visual_tools::RvizVisualToolsPtr left_row_plane_rviz_tools_;
  rviz_visual_tools::RvizVisualToolsPtr right_row_plane_rviz_tools_;

  void buildRobotPolygon();
  void object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
  void rowObjectsCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void checkCollisions();
  void computeRowsPaths();
  std::pair<double, double> lineCoefficientsWithRowLineSegmentation(const sensor_msgs::PointCloud2& pc2);
  std::pair<double, double> lineCoefficientsWithRowPlaneSegmentation(const sensor_msgs::PointCloud2& pc2,
                                                                     const std::string& row);
  nav_msgs::Path pathFromLine(double line_a, double line_b, double start_x, double end_x);
  void publishOutputPolygons();
  void publishDetectionStatus();
  void sendProcessingTimeInfo(double time_ms);
  void dynamicReconfigureCallback(const agribot_obstacle_detection::ObstaclesArbiterConfig& dynamic_config,
                                  uint32_t level);

public:
  ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag,
                       std::unique_ptr<IPolygonCollisionDetection> collision_detection);
  void run() const;
};
