/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/ObjectPolygon.h>

#include <algorithm>
#include <vector>

struct IPolygonCollisionDetection
{
  IPolygonCollisionDetection() = default;
  virtual ~IPolygonCollisionDetection() = default;
  IPolygonCollisionDetection(const IPolygonCollisionDetection& other) = delete;
  IPolygonCollisionDetection(IPolygonCollisionDetection&& other) = delete;
  IPolygonCollisionDetection& operator=(const IPolygonCollisionDetection& other) = delete;
  IPolygonCollisionDetection& operator=(IPolygonCollisionDetection&& other) = delete;

  virtual bool areColliding(const agribot_msgs::ObjectPolygon& p1, const agribot_msgs::ObjectPolygon& p2) const = 0;
};
