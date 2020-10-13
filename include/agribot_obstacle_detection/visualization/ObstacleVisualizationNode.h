/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>

class ObstacleVisualizationNode
{
private:
  ros::NodeHandle* nh;

public:
  explicit ObstacleVisualizationNode(ros::NodeHandle* _nh);
  ~ObstacleVisualizationNode();

  void run(void);
};
