/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <ros/ros.h>

#include "agribot_obstacle_detection/arbiter/ObstacleArbiterNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_arbiter_node");
  ros::NodeHandle nh("~");
  ObstacleArbiterNode oan(&nh);
  oan.run();
}
