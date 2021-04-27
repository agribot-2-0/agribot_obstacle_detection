/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "agribot_obstacle_detection/LidarPointCloudPipelineConfig.h"
#include "agribot_obstacle_detection/point_cloud_pipeline/PointCloudPipelineNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_point_cloud_pipeline_node");
  ros::NodeHandle nh("~");
  PointCloudPipelineNode<pcl::PointXYZI, agribot_obstacle_detection::LidarPointCloudPipelineConfig> pcpn(&nh);
  pcpn.run();
}
