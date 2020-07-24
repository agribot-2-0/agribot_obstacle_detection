/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <agribot_msgs/SonarStatus.h>
#include <agribot_msgs/SonarDetection.h>
#include <sensor_msgs/Range.h>

#include <unordered_map>
#include <string>
#include <memory>

#include "agribot_obstacle_detection/sonar/sonar.h"

class SonarObstacleDetectionNode
{
private:
  // ROS handlers
  ros::NodeHandle* nh;
  ros::Publisher sonarDetectionPub;

  // Sonars map with id as a key
  std::unordered_map<std::string, Sonar> sonars;

  // Subscribers map with id as a key
  std::unordered_map<std::string, std::shared_ptr<ros::Subscriber>> subscribers;

  // SonarDetection message
  agribot_msgs::SonarDetection sonarDetectionMsg;

public:
  explicit SonarObstacleDetectionNode(ros::NodeHandle* _nh);
  ~SonarObstacleDetectionNode();

  void sonarCallback(const sensor_msgs::Range::ConstPtr& msg, const std::string& id);

  void updateSonarDetectionMsg(const std::string& id, const std::string& frameId, ros::Time stamp, uint8_t status,
                               float shortestRange, float lastRange);

  void publishSonarDetecionMsg();
  void registerSubscribers();
  void initSonarDetectionMsg();
  void loadSonars();
  void run();
};
