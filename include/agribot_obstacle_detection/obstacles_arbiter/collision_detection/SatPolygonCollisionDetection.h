/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/ObjectPolygon.h>

#include <algorithm>
#include <vector>

#include "agribot_obstacle_detection/obstacles_arbiter/collision_detection/Projection.h"
#include "agribot_obstacle_detection/util/Conversions.h"

// Inspired by http://www.dyn4j.org/2010/01/sat

class SatPolygonCollisionDetection
{
public:
  bool areColliding(const agribot_msgs::ObjectPolygon& p1, const agribot_msgs::ObjectPolygon& p2) const;

private:
  std::vector<Eigen::Vector2f> getNormalAxes(const agribot_msgs::ObjectPolygon& p) const;
  std::vector<Eigen::Vector2f> getNormalAxes(const agribot_msgs::ObjectPolygon& p1,
                                             const agribot_msgs::ObjectPolygon& p2) const;
  Eigen::Vector2f getNormalAxis(const Eigen::Vector2f& vector) const;
  std::vector<Projection> getPolygonProjectionsOnAxes(const agribot_msgs::ObjectPolygon& p,
                                                      const std::vector<Eigen::Vector2f>& axes) const;
  Projection getPolygonProjectionOnAxis(const agribot_msgs::ObjectPolygon& p, const Eigen::Vector2f& axis) const;
};
