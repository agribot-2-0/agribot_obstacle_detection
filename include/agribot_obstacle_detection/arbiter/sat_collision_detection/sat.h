#pragma once

#include <eigen3/Eigen/Core>
#include <agribot_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>

#include "agribot_obstacle_detection/arbiter/sat_collision_detection/Projection.h"

namespace SAT
{
bool intersect(const agribot_msgs::PolygonStamped& firstPolygon, const agribot_msgs::PolygonStamped& secondPolygon);
Projection getProjection(const agribot_msgs::PolygonStamped& polygon, const Eigen::Vector2f& axis);
std::vector<Projection> getProjections(const agribot_msgs::PolygonStamped& polygon,
                                       const std::vector<Eigen::Vector2f>& axes);
std::vector<Eigen::Vector2f> getNormalAxes(const agribot_msgs::PolygonStamped& polygon);
Eigen::Vector2f toEigenVector2f(const geometry_msgs::Point& point);
geometry_msgs::Point toRosPoint(const Eigen::Vector2f& pointVector);

}  // namespace SAT
