/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/ObjectPolygon.h>

// Inspired by http://www.dyn4j.org/2010/01/sat

class SatPolygonCollisionDetection
{
public:
  bool areColliding(const agribot_msgs::ObjectPolygon& p1, const agribot_msgs::ObjectPolygon& p2);
};
