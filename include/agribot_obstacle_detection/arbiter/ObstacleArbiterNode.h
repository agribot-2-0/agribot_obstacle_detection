#pragma once

#include <ros/ros.h>
#include <agribot_msgs/AgribotObstacleDetection.h>
#include <agribot_msgs/SonarStatus.h>
#include <agribot_msgs/SonarDetection.h>
#include <agribot_msgs/DetectedObjectsArray.h>

class ObstacleArbiterNode
{
 private:
    ros::NodeHandle *nh;

    // Obstalce detection publisher
    ros::Publisher obstacleDetectionPub;

    // Sonar & Lidar detection subscribers
    ros::Subscriber sonarDetectionSub;
    ros::Subscriber detectedObjectsArraySub;

    // Detection status
    uint8_t status;
    uint8_t sonarStatus;
    uint8_t lidarStatus;

 public:
    explicit ObstacleArbiterNode(ros::NodeHandle *_nh);
    ~ObstacleArbiterNode();

    void sonarDetectionCallback(const agribot_msgs::SonarDetection::ConstPtr& msg);
    void detectedObjectsArrayCallback(const agribot_msgs::DetectedObjectsArray::ConstPtr& msg);
    void updateDetectionStatus();
    void publishDetectionStatus();
    void run();
};
