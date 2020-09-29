/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <libpcan.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>

class PCanSonars
{
private:
  // CAN addresses
  static constexpr uint16_t balluffMsgId = 4096;

  // ROS handlers
  ros::NodeHandle* nh;
  ros::Publisher balluffSonarPub;
  HANDLE canHandle;


public:
  explicit PCanSonars(ros::NodeHandle* _nh);
  void init_can();
  void init_ros(int argc, char** argv);
  void run();
  void read_can();

  float get_distance_from_balluff_sonar_msg(TPCANRdMsg* m);
};
