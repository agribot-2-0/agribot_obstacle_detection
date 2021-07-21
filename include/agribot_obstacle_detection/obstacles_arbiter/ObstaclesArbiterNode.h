/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <pcl/segmentation/sac_segmentation.h>
#pragma GCC diagnostic pop

#include <memory>
#include <optional>
#include <string>

#include "agribot_common/CommonFunctions/ros_common_functions.h"
#include "agribot_msgs/Object3D.h"
#include "agribot_msgs/Object3DArray.h"
#include "agribot_msgs/ObjectPolygon.h"
#include "agribot_msgs/ObjectPolygonArray.h"
#include "agribot_msgs/ObstacleDetectionStatus.h"
#include "agribot_obstacle_detection/obstacles_arbiter/collision_detection/IPolygonCollisionDetection.h"
#include "agribot_obstacle_detection/util/Conversions.h"
#include "agribot_obstacle_detection/util/ProcessingTimer.h"

class ObstaclesArbiterNode
{
private:
  double safety_zone_length_;
  double safety_zone_width_;
  std::string objects_target_frame_id_;
  bool publish_processing_time_;
  bool print_processing_time_;

  std::optional<agribot_msgs::ObjectPolygonArray> path_polygons_;
  std::optional<agribot_msgs::ObjectPolygonArray> objects_polygons_;
  agribot_msgs::ObjectPolygonArray output_polygons_;
  agribot_msgs::ObstacleDetectionStatus detection_status_;

  ros::NodeHandle* nh_;

  // Main subscribers and publishers
  ros::Subscriber in_object_array_sub_;
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

  // Logging and debug
  std::string log_tag_;
  ProcessingTimer timer;

  std::unique_ptr<IPolygonCollisionDetection> collision_detection_;

  void object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& msg);
  void checkCollisions();
  void computeRowsPaths();
  void publishOutputPolygons();
  void publishDetectionStatus();
  void sendProcessingTimeInfo(double time_ms);

public:
  ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag,
                       std::unique_ptr<IPolygonCollisionDetection> collision_detection);
  void run() const;
};
