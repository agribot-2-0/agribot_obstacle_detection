/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

struct PlaneCoefficients
{
  double a_;
  double b_;
  double c_;
  double d_;

  PlaneCoefficients() = delete;
  PlaneCoefficients(double a, double b, double c, double d) : a_{ a }, b_{ b }, c_{ c }, d_{ d } {};
};
