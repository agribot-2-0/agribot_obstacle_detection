/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <ros/ros.h>

#include "agribot_obstacle_detection/visualization/ObstacleVisualizationNode.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_visualization_node");
  ros::NodeHandle nh("~");
  ObstacleVisualizationNode ovn(&nh);
  ovn.run();
}
