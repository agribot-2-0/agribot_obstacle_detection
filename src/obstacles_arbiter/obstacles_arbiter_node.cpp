/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <ros/ros.h>

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacles_arbiter_node");
  ros::NodeHandle nh("~");
  std::string log_tag = "[ObstaclesArbiterNode] ";
  ObstaclesArbiterNode oan(&nh, log_tag);
  oan.run();
}
