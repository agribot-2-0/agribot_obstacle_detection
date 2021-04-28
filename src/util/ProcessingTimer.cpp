/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/util/ProcessingTimer.h"

void ProcessingTimer::start()
{
  start_time_ = std::chrono::steady_clock::now();
}

void ProcessingTimer::stop()
{
  end_time_ = std::chrono::steady_clock::now();
}

double ProcessingTimer::get_final_elapsed_time_s() const
{
  int64_t time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count();
  return microseconds_to_seconds(time_us);
}

double ProcessingTimer::get_final_elapsed_time_ms() const
{
  int64_t time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time_ - start_time_).count();
  return microseconds_to_miliseconds(time_us);
}

double ProcessingTimer::microseconds_to_seconds(int64_t time_us)
{
  return static_cast<double>(time_us) / 1000000.0;
}

double ProcessingTimer::microseconds_to_miliseconds(int64_t time_us)
{
  return static_cast<double>(time_us) / 1000.0;
}
