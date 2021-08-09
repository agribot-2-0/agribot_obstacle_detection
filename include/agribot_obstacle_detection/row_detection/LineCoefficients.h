/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

struct LineCoefficients
{
  double a_;
  double b_;

  LineCoefficients() = delete;
  LineCoefficients(double a, double b) : a_{ a }, b_{ b } {};
};
