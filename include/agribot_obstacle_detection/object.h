#pragma once

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

#include "agribot_obstacle_detection/bounding_box.h"

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

    void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, int id)
    {
        namespace vis = ::pcl::visualization;

        std::string idString = std::to_string(id);
        std::string objectCloudId = "objectCloud_" + idString;
        std::string objectTextId = "objectText_" + idString;
        std::string caption = "Object_" + idString;

        // Add Object cloud
        vis::PointCloudColorHandlerCustom<PointT> objectCloudRed(objectCloud,
                                                                 255, 0, 0);
        viewer->addPointCloud(objectCloud, objectCloudRed, objectCloudId);
        viewer->setPointCloudRenderingProperties(vis::PCL_VISUALIZER_OPACITY,
                                                1.0,
                                                objectCloudId);
        viewer->setPointCloudRenderingProperties(vis::PCL_VISUALIZER_POINT_SIZE,
                                                2.0,
                                                objectCloudId);

        // Add Object text
        float textScale = 0.2;
        float r = 1.0, g = 1.0, b = 1.0;
        PointT center;
        pcl::computeCentroid<PointT>(*objectCloud, center);
        viewer->addText3D(caption, center, textScale, r, g, b, objectTextId);

        // Add bounding box
        box.visualize(viewer, id);
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
