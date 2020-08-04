/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>
#include <string>
#include <ostream>
#include <algorithm>

class Sonar
{
private:
  // Threshold for determination of obstacle presence
  float threshold;

  // Vector keeping history of range measurements
  std::vector<float> ranges;

  // Number of samples to be kept in history
  std::size_t historySize;

  // Sonar identifier
  std::string id;

  // Sonar topic
  std::string topic;

public:
  explicit Sonar(float threshold, std::size_t historySize = 1, const std::string& id = "",
                 const std::string& topic = "");
  ~Sonar();

  // Add range measurement to the vector
  void addRange(float newRange);

  bool isObstacleDetected() const;

  float getThreshold() const;
  float getShortestRange() const;
  float getLastRange() const;
  std::string getId() const;
  std::string getTopic() const;

  friend std::ostream& operator<<(std::ostream& os, const Sonar& sonar);
};
