#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

#include "agribot_obstacle_detection/arbiter/sat_collision_detection/Projection.h"

class Polygon
{
 private:
    std::vector<Eigen::Vector2f> points;
 public:
    Polygon();
    ~Polygon();
    void addPoint(const Eigen::Vector2f& point);
    void addPoint(float x, float y);
    std::vector<Eigen::Vector2f> getNormalAxes() const;
    std::vector<Projection> getProjections(const std::vector<Eigen::Vector2f>& axes) const;
    Projection getProjection(const Eigen::Vector2f& axis) const;
    bool intersect(const Polygon& other) const;
};
