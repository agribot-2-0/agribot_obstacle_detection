/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/sonar/SonarObstacleDetectionNode.h"

SonarObstacleDetectionNode::SonarObstacleDetectionNode(ros::NodeHandle* _nh) : nh(_nh)
{
  // Create sonar detection publisher with latched connection
  sonarDetectionPub = nh->advertise<agribot_msgs::SonarDetection>("/agribot/sonar_detection", 1, true);

  // Load sonar parameters
  loadSonars();

  // Create initial SonarDetection message based on loaded sonars
  initSonarDetectionMsg();
  // Publish initial msg with WAITING_FOR_FIRST_MEASUREMENT status
  publishSonarDetecionMsg();

  // Register sonar subscribers
  registerSubscribers();
}

SonarObstacleDetectionNode::~SonarObstacleDetectionNode()
{
}

void SonarObstacleDetectionNode::sonarCallback(const sensor_msgs::Range::ConstPtr& msg, const std::string& id)
{
  // Add range to sonar detector object assigned to id
  auto sonar = sonars.find(id);
  sonar->second.addRange(msg->range);

  // Check if obstacle is present in any of measurements
  uint8_t status;
  if (sonar->second.isObstacleDetected())
  {
    status = agribot_msgs::SonarStatus::OBSTACLE_DETECTED;
  }
  else
  {
    status = agribot_msgs::SonarStatus::NO_OBSTACLE;
  }

  // Get shortest and last range of the sonar
  float shortestRange = sonar->second.getShortestRange();
  float lastRange = sonar->second.getLastRange();

  // Update and publish SonarDetection message
  updateSonarDetectionMsg(id, msg->header.frame_id, msg->header.stamp, status, shortestRange, lastRange,
                          msg->field_of_view);
  publishSonarDetecionMsg();
}

void SonarObstacleDetectionNode::updateSonarDetectionMsg(const std::string& id, const std::string& frameId,
                                                         ros::Time stamp, uint8_t status, float shortestRange,
                                                         float lastRange, float fieldOfView)
{
  // Update status for sonar matching id
  for (auto& it : sonarDetectionMsg.sonars)
  {
    if (it.id == id)
    {
      it.header.stamp = stamp;
      it.header.frame_id = frameId;
      it.status = status;
      it.shortest_range = shortestRange;
      it.last_range = lastRange;
      it.field_of_view = fieldOfView;
      break;
    }
  }
}

void SonarObstacleDetectionNode::publishSonarDetecionMsg()
{
  sonarDetectionPub.publish(sonarDetectionMsg);
}

void SonarObstacleDetectionNode::registerSubscribers()
{
  for (const auto& [id, sonar] : sonars)
  {
    std::string topic = sonar.getTopic();

    // Create subscriber ptr to avoid copying, when inserting into map
    std::shared_ptr<ros::Subscriber> sub;
    sub = std::make_shared<ros::Subscriber>();

    // Create boost::bind callback function, which passes
    // appropriate id string, used to determine source
    // of incoming measurement
    auto boostSonarCallback = boost::bind(&SonarObstacleDetectionNode::sonarCallback, this, _1, id);

    // Subscribe with boostSonarCallback function
    *sub = nh->subscribe<sensor_msgs::Range>(topic, 1, boostSonarCallback);

    // Add subscriber to subscribers map
    std::pair<std::string, std::shared_ptr<ros::Subscriber>> subPair(topic, sub);
    subscribers.insert(subPair);

    // Print information
    ROS_INFO_STREAM("[Sonar Obstacle Detection] Subscribed to topic: " << topic);
  }
}

void SonarObstacleDetectionNode::initSonarDetectionMsg()
{
  // Add entry to sonarDetectionMsg for every laoded sonar
  for (const auto& [id, sonar] : sonars)
  {
    agribot_msgs::SonarStatus sonarStatusMsg;
    sonarStatusMsg.id = sonar.getId();
    sonarStatusMsg.status = agribot_msgs::SonarStatus::WAITING_FOR_FIRST_MEASUREMENT;
    sonarStatusMsg.threshold = sonar.getThreshold();
    sonarDetectionMsg.sonars.push_back(sonarStatusMsg);
  }
}

void SonarObstacleDetectionNode::loadSonars()
{
  bool allParametersLoaded = false;
  std::size_t i = 0;

  std::string sonarField, thresholdField, topicField, historyLengthField;

  float threshold;
  int historyLength;
  std::string id, topic;

  // Check all configurations starting from sonar0
  while (!allParametersLoaded)
  {
    sonarField = "sonar" + std::to_string(i);
    topicField = sonarField + "_topic";
    thresholdField = sonarField + "_threshold";
    historyLengthField = sonarField + "_history_length";

    if (nh->hasParam(sonarField))
    {
      if (nh->hasParam(topicField) && nh->hasParam(thresholdField) && nh->hasParam(historyLengthField))
      {
        // Load all parameters for a sonar
        nh->getParam(sonarField, id);
        nh->getParam(topicField, topic);
        nh->getParam(thresholdField, threshold);
        nh->getParam(historyLengthField, historyLength);

        // Add sonar to sonars map
        Sonar sonar(threshold, historyLength, id, topic);
        std::pair<std::string, Sonar> sonarPair(id, sonar);
        sonars.insert(sonarPair);

        ++i;
      }
      else
      {
        // Error in configuration - no threshold for this sonar
        ROS_ERROR_STREAM("[Sonar Obstacle Detection] " << topicField << ", " << thresholdField << " or "
                                                       << historyLengthField << " unspecified for " << sonarField
                                                       << ". Exiting...");

        ros::shutdown();
      }
    }
    else
    {
      allParametersLoaded = true;
    }
  }

  ROS_INFO_STREAM("[Sonar Obstacle Detection] Loaded " << i << " sonars.");
}

void SonarObstacleDetectionNode::run()
{
  ros::spin();
}
