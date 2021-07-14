/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

class Projection
{
private:
  double min_;
  double max_;

public:
  Projection(double min, double max);
  Projection() = delete;
  bool overlap(const Projection& other) const;
};
