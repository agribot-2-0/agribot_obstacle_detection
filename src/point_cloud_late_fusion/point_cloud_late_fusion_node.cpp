/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "agribot_obstacle_detection/point_cloud_late_fusion/PointCloudLateFusionNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_late_fusion_node");
  ros::NodeHandle nh("~");
  std::string log_tag = "[PointCloudLateFusionNode] ";
  PointCloudLateFusionNode pclfn(&nh, log_tag);
  pclfn.run();
}
