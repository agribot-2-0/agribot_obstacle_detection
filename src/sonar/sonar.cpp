#include "agribot_obstacle_detection/sonar/sonar.h"

Sonar::Sonar(float setThreshold, std::size_t setHistorySize, const std::string& setId, const std::string& setTopic)
{
  threshold = setThreshold;
  historySize = setHistorySize;
  id = setId;
  topic = setTopic;
}

Sonar::~Sonar()
{
}

void Sonar::addRange(float newRange)
{
  // Remove oldest measurement to keep only 'historySize' ranges
  if (ranges.size() == historySize)
  {
    ranges.erase(ranges.begin());
  }
  // Add new measurement to the back of the vector
  ranges.push_back(newRange);
}

bool Sonar::isObstacleDetected() const
{
  for (const auto& it : ranges)
  {
    if (it < threshold)
    {
      // Obstacle detected in at least one of measurements
      return true;
    }
  }

  // No obstacle detected
  return false;
}

float Sonar::getThreshold() const
{
  return threshold;
}

float Sonar::getShortestRange() const
{
  return *std::min_element(ranges.begin(), ranges.end());
}

float Sonar::getLastRange() const
{
  return ranges.back();
}

std::string Sonar::getId() const
{
  return id;
}

std::string Sonar::getTopic() const
{
  return topic;
}

// Ostream for debugging purposes
std::ostream& operator<<(std::ostream& os, const Sonar& sonar)
{
  os << std::endl
     << "Sonar: " << sonar.id << std::endl
     << "Threshold: " << sonar.threshold << std::endl
     << "History size: " << sonar.historySize << std::endl
     << "Measurements in history: " << sonar.ranges.size() << std::endl
     << "\t[From oldest to newest]" << std::endl;

  for (const auto& it : sonar.ranges)
  {
    os << "\tRange: " << it << std::endl;
  }

  return os;
}
