/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <ros/ros.h>

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"
#include "agribot_obstacle_detection/obstacles_arbiter/collision_detection/SatPolygonCollisionDetection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacles_arbiter_node");
  ros::NodeHandle nh("~");
  std::string log_tag = "[ObstaclesArbiterNode] ";

  std::unique_ptr<IPolygonCollisionDetection> collision_detection = std::make_unique<SatPolygonCollisionDetection>();

  try
  {
    ObstaclesArbiterNode oan(&nh, log_tag, std::move(collision_detection));
    oan.run();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM(log_tag << e.what());
    ros::shutdown();
    exit(-1);
  }
}
