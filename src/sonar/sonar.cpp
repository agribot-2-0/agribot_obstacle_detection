/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#include "agribot_obstacle_detection/sonar/sonar.h"

Sonar::Sonar(float setThreshold,
             std::size_t setHistorySize,
             std::string setId,
             std::string setTopic)
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

bool Sonar::isObstacleDetected()
{
    for (auto it = ranges.begin(); it != ranges.end(); ++it)
    {
        if (*it < threshold)
        {
            // Obstacle detected in at least one of measurements
            return true;
        }
    }

    // No obstacle detected
    return false;
}

float Sonar::getThreshold()
{
    return threshold;
}

float Sonar::getShortestRange()
{
    return *std::min_element(ranges.begin(), ranges.end());
}

float Sonar::getLastRange()
{
    return ranges.back();
}

std::string Sonar::getId()
{
    return id;
}

std::string Sonar::getTopic()
{
    return topic;
}


// Ostream for debugging purposes
std::ostream &operator<<(std::ostream &os, Sonar const &sonar)
{
    os << std::endl << "Sonar: " << sonar.id << std::endl
       << "Threshold: " << sonar.threshold << std::endl
       << "History size: " << sonar.historySize << std::endl
       << "Measurements in history: " << sonar.ranges.size() << std::endl
       << "\t[From oldest to newest]" << std::endl;

    for (auto it = sonar.ranges.begin(); it != sonar.ranges.end(); ++it)
    {
        os << "\tRange: " << *it << std::endl;
    }

    return os;
}
