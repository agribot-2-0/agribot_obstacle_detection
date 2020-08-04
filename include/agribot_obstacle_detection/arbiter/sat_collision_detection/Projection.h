/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

class Projection
{
private:
  float min;
  float max;

public:
  Projection(float minValue, float maxValue);
  ~Projection();
  bool overlap(const Projection& other) const;
};
