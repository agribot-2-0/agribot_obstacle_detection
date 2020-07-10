#include <eigen3/Eigen/Core>
#include <iostream>
#include "agribot_obstacle_detection/arbiter/sat_collision_detection/Polygon.h"

int main(int argc, char **argv)
{
    std::vector<Eigen::Vector2f> firstPolyPoints;
    firstPolyPoints.push_back(Eigen::Vector2f(0, 0));
    firstPolyPoints.push_back(Eigen::Vector2f(5, 0));
    firstPolyPoints.push_back(Eigen::Vector2f(5, 5));
    firstPolyPoints.push_back(Eigen::Vector2f(0, 5));

    // firstPolyPoints.push_back(Eigen::Vector2f(0, 0));
    // firstPolyPoints.push_back(Eigen::Vector2f(5, 0));
    // firstPolyPoints.push_back(Eigen::Vector2f(5, 5));
    // firstPolyPoints.push_back(Eigen::Vector2f(0, 5));

    std::vector<Eigen::Vector2f> secondPolyPoints;
    secondPolyPoints.push_back(Eigen::Vector2f(6, 2));
    secondPolyPoints.push_back(Eigen::Vector2f(3, 11.2));
    secondPolyPoints.push_back(Eigen::Vector2f(3, 12));
    secondPolyPoints.push_back(Eigen::Vector2f(6, 3));

    // secondPolyPoints.push_back(Eigen::Vector2f(6, 2));
    // secondPolyPoints.push_back(Eigen::Vector2f(3, 11.2));
    // secondPolyPoints.push_back(Eigen::Vector2f(3, 12));
    // secondPolyPoints.push_back(Eigen::Vector2f(6, 3));

    Polygon poly1;
    for (const auto& point : firstPolyPoints)
    {
        // Add point to polygon
        poly1.addPoint(point);

        // Print polygon coordinates
        std::cout << point[0] << " " << point[1] << std::endl;
    }
    std::cout << firstPolyPoints.front()[0] << " " << firstPolyPoints.front()[1] << std::endl;


    // Aditional empty lines to separate polygons
    std::cout << std::endl;

    Polygon poly2;
    for (const auto& point : secondPolyPoints)
    {
        // Add point to polygon
        poly2.addPoint(point);

        // Print polygon coordinates
        std::cout << point[0] << " " << point[1] << std::endl;
    }
    std::cout << secondPolyPoints.front()[0] << " " << secondPolyPoints.front()[1] << std::endl;


    if (poly1.intersect(poly2))
    {
        std::cerr << "Polygons intersect" << std::endl;
    }
    else
    {
        std::cerr << "NO intersection" << std::endl;
    }

    if (poly2.intersect(poly1))
    {
        std::cerr << "Polygons intersect" << std::endl;
    }
    else
    {
        std::cerr << "NO intersection" << std::endl;
    }
}
