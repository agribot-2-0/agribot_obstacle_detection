/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

class Projection
{
private:
  float min_;
  float max_;

public:
  Projection(float min, float max);
  bool overlap(const Projection& other) const;
};
