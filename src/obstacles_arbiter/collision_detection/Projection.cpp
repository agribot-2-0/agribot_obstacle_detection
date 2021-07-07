/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/collision_detection/Projection.h"

Projection::Projection(float min, float max) : min_{ min }, max_{ max }
{
}

bool Projection::overlap(const Projection& other) const
{
  if ((min_ >= other.min_) && (min_ <= other.max_))
  {
    return true;
  }
  else if ((max_ >= other.min_) && (max_ <= other.max_))
  {
    return true;
  }
  else if ((other.min_ >= min_) && (other.min_ <= max_))
  {
    return true;
  }
  else if ((other.max_ >= min_) && (other.max_ <= max_))
  {
    return true;
  }
  else
  {
    // No overlap condition met
    return false;
  }
}
