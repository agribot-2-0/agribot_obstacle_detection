/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_msgs/PolygonStamped.h>

#include <eigen3/Eigen/Core>
#include <iostream>

#include "agribot_obstacle_detection/arbiter/sat_collision_detection/sat.h"

int main(int argc, char** argv)
{
  std::vector<Eigen::Vector2f> firstPolyPoints;
  firstPolyPoints.push_back(Eigen::Vector2f(0, 0));
  firstPolyPoints.push_back(Eigen::Vector2f(5, 0));
  firstPolyPoints.push_back(Eigen::Vector2f(5, 5));
  firstPolyPoints.push_back(Eigen::Vector2f(0, 5));

  std::vector<Eigen::Vector2f> secondPolyPoints;
  secondPolyPoints.push_back(Eigen::Vector2f(6, 2));
  secondPolyPoints.push_back(Eigen::Vector2f(3, 11.2));
  secondPolyPoints.push_back(Eigen::Vector2f(3, 12));
  secondPolyPoints.push_back(Eigen::Vector2f(6, 3));

  agribot_msgs::PolygonStamped poly1;
  for (const auto& point : firstPolyPoints)
  {
    // Add point to polygon
    poly1.polygon.points.push_back(SAT::toRosPoint(point));

    // Print polygon coordinates
    std::cout << point[0] << " " << point[1] << std::endl;
  }
  std::cout << firstPolyPoints.front()[0] << " " << firstPolyPoints.front()[1] << std::endl;

  // Aditional empty lines to separate polygons
  std::cout << std::endl;

  agribot_msgs::PolygonStamped poly2;
  for (const auto& point : secondPolyPoints)
  {
    // Add point to polygon
    poly2.polygon.points.push_back(SAT::toRosPoint(point));

    // Print polygon coordinates
    std::cout << point[0] << " " << point[1] << std::endl;
  }
  std::cout << secondPolyPoints.front()[0] << " " << secondPolyPoints.front()[1] << std::endl;

  if (SAT::intersect(poly1, poly2))
  {
    std::cerr << "Polygons intersect" << std::endl;
  }
  else
  {
    std::cerr << "NO intersection" << std::endl;
  }

  if (SAT::intersect(poly2, poly1))
  {
    std::cerr << "Polygons intersect" << std::endl;
  }
  else
  {
    std::cerr << "NO intersection" << std::endl;
  }
}
