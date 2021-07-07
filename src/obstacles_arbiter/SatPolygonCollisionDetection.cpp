/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/SatPolygonCollisionDetection.h"

bool SatPolygonCollisionDetection::areColliding(const agribot_msgs::ObjectPolygon& p1,
                                                const agribot_msgs::ObjectPolygon& p2) const
{
  // Get projection axes for both polygons
  auto normal_axes = getNormalAxes(p1, p2);

  // Project both polygons on normal axes
  auto p1_projections = getPolygonProjectionsOnAxes(p1, normal_axes);
  auto p2_projections = getPolygonProjectionsOnAxes(p2, normal_axes);

  for (std::size_t i = 0; i < p1_projections.size(); i++)
  {
    if (!p1_projections[i].overlap(p2_projections[i]))
    {
      // If any of the projections pairs is not overlapping, polygons are not intersecting
      return false;
    }
  }

  // If all projections overlap, polygons are intersecting
  return true;
}

std::vector<Eigen::Vector2f> SatPolygonCollisionDetection::getNormalAxes(const agribot_msgs::ObjectPolygon& p) const
{
  std::vector<Eigen::Vector2f> axes;

  /* General case - axes from consecutive points */
  for (auto it = p.polygon.points.begin(); it != std::prev(p.polygon.points.end()); ++it)
  {
    // Get edge between next and current point
    Eigen::Vector2f current_point = Conversions::convertToEigenVector2f(*it);
    Eigen::Vector2f next_point = Conversions::convertToEigenVector2f(*std::next(it));
    Eigen::Vector2f normal_axis = getNormalAxis(next_point - current_point);
    axes.push_back(normal_axis.normalized());
  }

  /* Special case - additional edge between first and last point */
  Eigen::Vector2f current_point = Conversions::convertToEigenVector2f(p.polygon.points.back());
  Eigen::Vector2f next_point = Conversions::convertToEigenVector2f(p.polygon.points.front());
  Eigen::Vector2f normal_axis = getNormalAxis(next_point - current_point);
  axes.push_back(normal_axis.normalized());

  return axes;
}

std::vector<Eigen::Vector2f> SatPolygonCollisionDetection::getNormalAxes(const agribot_msgs::ObjectPolygon& p1,
                                                                         const agribot_msgs::ObjectPolygon& p2) const
{
  auto v1 = getNormalAxes(p1);
  auto v2 = getNormalAxes(p2);

  // Combine v1 and v2 into one vector
  v1.reserve(v1.size() + v2.size());
  v1.insert(v1.end(), v2.begin(), v2.end());

  return v1;
}

Eigen::Vector2f SatPolygonCollisionDetection::getNormalAxis(const Eigen::Vector2f& vector) const
{
  return Eigen::Vector2f(vector[1], -vector[0]);
}

std::vector<Projection> SatPolygonCollisionDetection::getPolygonProjectionsOnAxes(
    const agribot_msgs::ObjectPolygon& p, const std::vector<Eigen::Vector2f>& axes) const
{
  std::vector<Projection> projections;

  for (const auto& axis : axes)
  {
    projections.push_back(getPolygonProjectionOnAxis(p, axis));
  }

  return projections;
}

Projection SatPolygonCollisionDetection::getPolygonProjectionOnAxis(const agribot_msgs::ObjectPolygon& p,
                                                                    const Eigen::Vector2f& axis) const
{
  std::vector<float> point_projections;

  for (const auto& p : p.polygon.points)
  {
    Eigen::Vector2f point = Conversions::convertToEigenVector2f(p);
    point_projections.push_back(axis.dot(point));
  }

  float min_projection = *std::min_element(point_projections.begin(), point_projections.end());
  float max_projection = *std::max_element(point_projections.begin(), point_projections.end());

  return Projection(min_projection, max_projection);
}
