#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <agribot_msgs/BoundingBox.h>
#include <agribot_msgs/DetectedObject.h>

#include <string>

#include "agribot_obstacle_detection/lidar/bounding_box.h"

template<typename PointT>
class Object
{
 private:
    typename pcl::PointCloud<PointT>::Ptr objectCloud;
    PointT centroidPoint;
    BoundingBox<PointT> box;
    uint32_t id;

 public:
    explicit Object(const typename pcl::PointCloud<PointT>::Ptr setObjectCloud)
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

    void computeCentroid()
    {
        pcl::computeCentroid(*objectCloud, centroidPoint);
    }

    void setId(uint32_t newId)
    {
        id = newId;
    }

    uint32_t getId() const
    {
        return id;
    }

    typename pcl::PointCloud<PointT>::Ptr getObjectCloud() const
    {
        return objectCloud;
    }

    BoundingBox<PointT> getBoundingBox() const
    {
        return box;
    }

    agribot_msgs::DetectedObject toROSMsg() const
    {
        agribot_msgs::DetectedObject objectMsg;

        // Set object position to cloud's centroid point
        objectMsg.pose.position.x = centroidPoint.x;
        objectMsg.pose.position.y = centroidPoint.y;
        objectMsg.pose.position.z = centroidPoint.z;

        // Set identity quaternion
        objectMsg.pose.orientation.x = 0.0;
        objectMsg.pose.orientation.y = 0.0;
        objectMsg.pose.orientation.z = 0.0;
        objectMsg.pose.orientation.w = 1.0;

        // Set bounding box
        objectMsg.box = box.toROSMsg();

        // Set object id
        objectMsg.id = id;

        return objectMsg;
    }
};
