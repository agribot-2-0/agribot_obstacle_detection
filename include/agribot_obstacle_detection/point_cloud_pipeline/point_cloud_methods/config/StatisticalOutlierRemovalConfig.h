/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

struct StatisticalOutlierRemovalConfig
{
  int neighbours_n_;
  float stddev_mul_;
};
