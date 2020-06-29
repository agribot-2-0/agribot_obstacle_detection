#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <agribot_msgs/BoundingBox.h>

#include <string>

template<typename PointT>
class BoundingBox
{
 public:
    float xMin;
    float yMin;
    float zMin;
    float xMax;
    float yMax;
    float zMax;

    float depth;
    float width;
    float height;

    BoundingBox():
        xMin(0.0), yMin(0.0), zMin(0.0),
        xMax(0.0), yMax(0.0), zMax(0.0)
    {
    }

    ~BoundingBox()
    {
    }

    void compute(typename pcl::PointCloud<PointT>::Ptr cluster)
    {
        PointT minPoint, maxPoint;
        pcl::getMinMax3D(*cluster, minPoint, maxPoint);

        xMin = minPoint.x;
        yMin = minPoint.y;
        zMin = minPoint.z;
        xMax = maxPoint.x;
        yMax = maxPoint.y;
        zMax = maxPoint.z;

        depth = xMax - xMin;
        width = yMax - yMin;
        height = zMax - zMin;
    }

    agribot_msgs::BoundingBox toROSMsg()
    {
        agribot_msgs::BoundingBox box;

        // Set min_point
        box.min_point.x = xMin;
        box.min_point.y = yMin;
        box.min_point.z = zMin;

        // Set max_point
        box.max_point.x = xMax;
        box.max_point.y = yMax;
        box.max_point.z = zMax;

        // Set depth, width and height
        box.depth = depth;
        box.width = width;
        box.height = height;

        return box;
    }
};
