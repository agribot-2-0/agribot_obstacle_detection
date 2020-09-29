/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "sonar_pcan_communication/pcan_sonars.h"

#include <ros/ros.h>
#include <fcntl.h>

PCanSonars::PCanSonars(ros::NodeHandle* _nh) : nh(_nh)
{
}

void PCanSonars::init_can()
{
  int nType = HW_PCI;
  __u32 dwPort = 0;
  __u16 wIrq = 0;
  __u16 wBTR0BTR1 = 0x031c;
  const char* szDevNode = "/dev/pcanusb32";

  canHandle = LINUX_CAN_Open(szDevNode, O_RDWR);

  if (!canHandle)
  {
    ROS_FATAL_STREAM("Peak CAN open error...");
    ros::shutdown();
    exit(0);
  }

  uint32_t error_status = CAN_Init(canHandle, wBTR0BTR1, CAN_INIT_TYPE_ST);
  if (error_status)
  {
    ROS_FATAL_STREAM("Peak CAN init error...");
    ros::shutdown();
    exit(0);
  }
}

void PCanSonars::init_ros(int argc, char** argv)
{
  balluffSonarPub = nh->advertise<sensor_msgs::Range>("/bumper/front/sonar/center", 1);
}

void PCanSonars::run()
{
  while (ros::ok())
  {
    read_can();
  }
}

void PCanSonars::read_can()
{
  TPCANRdMsg canMsg;

  if (LINUX_CAN_Read(canHandle, &canMsg))
  {
    ROS_WARN_STREAM("Peak CAN read error...");
    return;
  }

  if (canMsg.Msg.ID == balluffMsgId)
  {
    sensor_msgs::Range rangeMsg;
    rangeMsg.header.frame_id = "bumper_front_sonar_center_link";
    rangeMsg.header.stamp = ros::Time::now();
    rangeMsg.range = get_distance_from_balluff_sonar_msg(&canMsg);
    rangeMsg.field_of_view = 0.57;
    rangeMsg.min_range = 0.35;
    rangeMsg.max_range = 10.0;
    rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    balluffSonarPub.publish(rangeMsg);
  }
  else
  {
    ROS_INFO_STREAM("MSG_ID not recognized...");
  }
}

float PCanSonars::get_distance_from_balluff_sonar_msg(TPCANRdMsg* m)
{
  __u16 adcValue = m->Msg.DATA[0] | (m->Msg.DATA[1] << 8);
  float distance = 0.35 + (static_cast<float>(adcValue) / 700.0);
  return distance;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_pcan_communication");
  ros::NodeHandle nh("~");
  PCanSonars pcs(&nh);

  pcs.init_can();
  pcs.init_ros(argc, argv);
  pcs.run();
}
