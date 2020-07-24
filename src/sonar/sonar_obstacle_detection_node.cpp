/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include <ros/ros.h>

#include "agribot_obstacle_detection/sonar/SonarObstacleDetectionNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_obstacle_detection_node");
    ros::NodeHandle nh("~");
    SonarObstacleDetectionNode sodn(&nh);
    sodn.run();
}
