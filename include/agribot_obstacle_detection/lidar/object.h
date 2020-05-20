#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <string>

#include "agribot_obstacle_detection/lidar/bounding_box.h"

template<typename PointT>
class Object
{
 private:
    typename pcl::PointCloud<PointT>::Ptr objectCloud;
    BoundingBox<PointT> box;

 public:
    explicit Object(typename pcl::PointCloud<PointT>::Ptr setObjectCloud)
    {
        objectCloud = setObjectCloud;
    }

    ~Object()
    {
    }

    void computeBoundingBox()
    {
        box.compute(objectCloud);
    }

    typename pcl::PointCloud<PointT>::Ptr getObjectCloud()
    {
        return objectCloud;
    }

    BoundingBox<PointT> getBoundingBox()
    {
        return box;
    }
};
