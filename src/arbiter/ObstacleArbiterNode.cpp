#include "agribot_obstacle_detection/arbiter/ObstacleArbiterNode.h"

ObstacleArbiterNode::ObstacleArbiterNode(ros::NodeHandle *_nh) :
    nh(_nh),
    status(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT),
    sonarStatus(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT),
    visionStatus(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
{
    // Advertise latched obstacles detection publisher
    obstacleDetectionPub = nh->advertise<agribot_msgs::AgribotObstacleDetection>
        ("/agribot/obstacle_detected", 1, true);

    // Publish initial detection status (WAITING_FOR_FIRST_MEASUREMENT)
    publishDetectionStatus();

    // Subscribe to Sonar & Lidar detection status

    sonarDetectionSub = _nh->subscribe("/sonar_obstacle_detection_node/sonar_detection",
        1,
        &ObstacleArbiterNode::sonarDetectionCallback,
        this);

    visionDetectionSub = _nh->subscribe("/agribot/vision_detection",
        1,
        &ObstacleArbiterNode::visionDetectionCallback,
        this);
}

ObstacleArbiterNode::~ObstacleArbiterNode()
{
}

void ObstacleArbiterNode::sonarDetectionCallback(const agribot_msgs::SonarDetection::ConstPtr& msg)
{
    uint8_t detectedCount = 0;
    uint8_t waitingCount = 0;

    // Iterate over sonars
    for (auto sonar : msg->sonars)
    {
        if (sonar.status == agribot_msgs::SonarStatus::OBSTACLE_DETECTED)
        {
            // Break if any obstacle is detected
            ++detectedCount;
            break;
        }
        else if (sonar.status == agribot_msgs::SonarStatus::WAITING_FOR_FIRST_MEASUREMENT)
        {
            // Following sonars may have OBSTACLE_DETECTED status - can't break
            ++waitingCount;
        }
    }

    // Update sonarStatus
    if (detectedCount > 0)
    {
        sonarStatus = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;
    }
    else if (waitingCount > 0)
    {
        sonarStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
    }
    else
    {
        sonarStatus = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;
    }

    // Update general detection status and publish it
    updateDetectionStatus();
    publishDetectionStatus();
}

void ObstacleArbiterNode::visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg)
{

}

void ObstacleArbiterNode::updateDetectionStatus()
{
    if (sonarStatus == agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED ||
        visionStatus == agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED)
    {
        status = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;
    }
    else if (sonarStatus == agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT ||
             visionStatus == agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
    {
        status = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
    }
    else
    {
        status = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;
    }
}

void ObstacleArbiterNode::publishDetectionStatus()
{
    agribot_msgs::AgribotObstacleDetection msg;
    msg.status = status;
    msg.sonar_status = sonarStatus;
    msg.vision_status = visionStatus;
    obstacleDetectionPub.publish(msg);
}


void ObstacleArbiterNode::run()
{
    ros::spin();
}
