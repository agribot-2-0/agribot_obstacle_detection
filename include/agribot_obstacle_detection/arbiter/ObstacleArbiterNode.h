/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <agribot_msgs/AgribotObstacleDetection.h>
#include <agribot_msgs/SonarStatus.h>
#include <agribot_msgs/SonarDetection.h>
#include <agribot_msgs/VisionDetection.h>
#include <agribot_msgs/DetectedObject.h>
#include <agribot_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <vector>
#include <string>
#include <utility>
#include <cmath>

#include "agribot_obstacle_detection/arbiter/sat_collision_detection/sat.h"

class ObstacleArbiterNode
{
private:
  ros::NodeHandle* nh;

  // Obstalce detection publisher
  ros::Publisher obstacleDetectionPub;

  // Sonar & Lidar detection subscribers
  ros::Subscriber sonarDetectionSub;
  ros::Subscriber visionDetectionSub;
  ros::Subscriber predictedPathSub;

  // TF Buffer and listener
  tf2_ros::Buffer TFBuffer;
  tf2_ros::TransformListener TFListener;

  // TF between vision objects and predicted path
  geometry_msgs::TransformStamped visionToPredictedPathTransform;

  // Rviz visualization
  rviz_visual_tools::RvizVisualToolsPtr polygonsVisualTools;
  rviz_visual_tools::RvizVisualToolsPtr sonarDetectionVisualTools;

  float predictedPathTimeout;
  ros::Timer predictedPathTimeoutTimer;

  float visionTimeout;
  ros::Timer visionTimeoutTimer;

  float robotLength;
  float robotWidth;
  float widthOffset;

  std::string predictedPathFrameId;
  std::string visionFrameId;

  // Vector containing pairs of: polygon casted from vision object bounding box, and bool informing if this polygon
  // intersects with any of predictedPathPolygons
  std::vector<std::pair<agribot_msgs::PolygonStamped, bool>> visionPolygonsPairs;

  // Vector containing polygons representing projection of robot on path
  std::vector<agribot_msgs::PolygonStamped> predictedPathPolygons;

  // Detection status
  uint8_t status;
  uint8_t sonarStatus;
  uint8_t visionStatus;

  // Additional flag to distinguish between not receiving vision detection message and having no obstacles in received
  // message
  bool visionMsgReceived;

public:
  explicit ObstacleArbiterNode(ros::NodeHandle* _nh);
  ~ObstacleArbiterNode();

  // ROS callbacks
  void sonarDetectionCallback(const agribot_msgs::SonarDetection::ConstPtr& msg);
  void visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg);
  void predictedPathCallback(const visualization_msgs::Marker::ConstPtr& msg);

  // Vision detection
  void visionDetectionPipeline();
  void checkVisionCollisions();

  // Vision detection transformations
  void updateVisionToPredictedPathTransform();
  void updatePredictedPathPolygons(const std::vector<geometry_msgs::Point>& predictedPathPoints);
  void updateVisionPolygons(const std::vector<agribot_msgs::DetectedObject>& detectedObjects);
  agribot_msgs::PolygonStamped getRobotPolygon(geometry_msgs::Point centerPoint, float angle) const;
  agribot_msgs::PolygonStamped getVisionObjectPolygon(const agribot_msgs::DetectedObject& detectedObject) const;

  float getAngleBetweenPoints(geometry_msgs::Point firstPoint, geometry_msgs::Point secondPoint) const;
  agribot_msgs::PolygonStamped doTransform(const agribot_msgs::PolygonStamped& inPolygon,
                                           const geometry_msgs::TransformStamped& transform) const;

  // Predicted path timeout timer
  void createPredictedPathTimeoutTimer();
  void stopPredictedPathTimeoutTimer();
  void startPredictedPathTimeoutTimer();
  void restartPredictedPathTimeoutTimer();
  void predictedPathTimeoutCallback(const ros::TimerEvent& event);

  // Vision timeout timer
  void createVisionTimeoutTimer();
  void stopVisionTimeoutTimer();
  void startVisionTimeoutTimer();
  void restartVisionTimeoutTimer();
  void visionTimeoutCallback(const ros::TimerEvent& event);

  // General methods
  void updateDetectionStatus();
  void publishDetectionStatus();
  void loadParameters();

  // Visualization
  void initVisualization();
  void publishPolygonsMarkers();
  void clearPolygonsMarkers();
  void publishVisionPolygonsMarkers();
  void publishRobotPolygonsMarkers();
  void publishSonarDetectionMarkers(const std::vector<agribot_msgs::SonarStatus>& collidingSonarsFrameIds);
  void clearSonarDetectionMarkers();

  void run();
};
