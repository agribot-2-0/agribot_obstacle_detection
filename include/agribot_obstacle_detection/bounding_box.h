#pragma once

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


    void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, int id)
    {
        // using namespace pcl::visualization;
        namespace vis = ::pcl::visualization;

        int wireframe = vis::PCL_VISUALIZER_REPRESENTATION_WIREFRAME;
        int surface = vis::PCL_VISUALIZER_REPRESENTATION_SURFACE;

        // Parameters
        float opacity = 1.0;
        float r = 0.0, g = 1.0, b = 1.0;
        std::string boxName = "box_" + std::to_string(id);

        // Cube wireframe
        viewer->addCube(xMin, xMax, yMin, yMax, zMin, zMax,
                        r, g, b, boxName);
        viewer->setShapeRenderingProperties(vis::PCL_VISUALIZER_REPRESENTATION,
                                            wireframe,
                                            boxName);
        viewer->setShapeRenderingProperties(vis::PCL_VISUALIZER_OPACITY,
                                            opacity,
                                            boxName);

        // Cube surface
        std::string boxFillName = "boxFill_" + std::to_string(id);
        viewer->addCube(xMin, xMax, yMin, yMax, zMin, zMax,
                        r, g, b, boxFillName);
        viewer->setShapeRenderingProperties(vis::PCL_VISUALIZER_REPRESENTATION,
                                            surface,
                                            boxFillName);
        viewer->setShapeRenderingProperties(vis::PCL_VISUALIZER_OPACITY,
                                            0.3 * opacity,
                                            boxFillName);
    }
};
