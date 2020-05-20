#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

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
    }
};
