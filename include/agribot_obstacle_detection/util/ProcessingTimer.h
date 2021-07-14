/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <chrono>  // NOLINT [build/c++11]

class ProcessingTimer
{
private:
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  std::chrono::time_point<std::chrono::steady_clock> end_time_;

public:
  ProcessingTimer() = default;
  ~ProcessingTimer() = default;
  void start();
  void stop();
  double get_final_elapsed_time_s() const;
  double get_final_elapsed_time_ms() const;

  static double microseconds_to_seconds(int64_t time_us);
  static double microseconds_to_miliseconds(int64_t time_us);
};
