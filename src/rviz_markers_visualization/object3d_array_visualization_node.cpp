/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <ros/ros.h>

#include "agribot_obstacle_detection/rviz_markers_visualization/Object3DArrayVisualizationNode.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object3d_array_visualization_node");
  ros::NodeHandle nh("~");
  Object3DArrayVisualizationNode visualization_node(&nh);
  visualization_node.run();
}
