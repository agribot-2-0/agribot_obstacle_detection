/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <ros/ros.h>

#include "agribot_obstacle_detection/row_detection/RowDetectionNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "row_detection_node");
  ros::NodeHandle nh("~");
  std::string log_tag = "[RowDetectionNode] ";

  try
  {
    RowDetectionNode rdn(&nh, log_tag);
    rdn.run();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM(log_tag << e.what());
    ros::shutdown();
    exit(-1);
  }
}
