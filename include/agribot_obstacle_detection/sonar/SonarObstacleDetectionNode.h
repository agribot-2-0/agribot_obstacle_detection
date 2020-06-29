#pragma once

#include <ros/ros.h>
#include <agribot_msgs/SonarStatus.h>
#include <agribot_msgs/SonarDetection.h>
#include <sensor_msgs/Range.h>

#include <map>
#include <string>
#include <memory>

#include "agribot_obstacle_detection/sonar/sonar.h"


class SonarObstacleDetectionNode
{
 private:
    // ROS handlers
    ros::NodeHandle *nh;
    ros::Publisher sonarDetectionPub;

    // ROS variables
    ros::Time lastTimestamp;

    // Sonars map with id as a key
    std::map<std::string, Sonar> sonars;

    // Subscribers map with id as a key
    std::map<std::string, std::shared_ptr<ros::Subscriber>> subscribers;

    // SonarDetection message
    agribot_msgs::SonarDetection sonarDetectionMsg;

 public:
    explicit SonarObstacleDetectionNode(ros::NodeHandle *_nh);
    ~SonarObstacleDetectionNode();

    void sonarCallback(
        const sensor_msgs::Range::ConstPtr& msg,
        std::string id);

    void updateSonarDetectionMsg(std::string id,
        uint8_t status,
        float shortestRange,
        float lastRange);

    void publishSonarDetecionMsg();
    void updateLastTimestamp(ros::Time newTimestamp);
    void registerSubscribers();
    void initSonarDetectionMsg();
    void loadSonars();
    void run();
};
