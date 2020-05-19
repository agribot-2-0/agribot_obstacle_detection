#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <chrono>   // NOLINT [build/c++11]

#include "agribot_obstacle_detection/object.h"

template<typename PointT>
class CloudProcessor
{
 public:
    CloudProcessor()
    {
    }

    ~CloudProcessor()
    {
    }


    void extractInliersOutliers(
        typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
        typename pcl::PointCloud<PointT>::Ptr inliersCloud,
        typename pcl::PointCloud<PointT>::Ptr outliersCloud,
        pcl::PointIndices::Ptr inliers)
    {
        pcl::ExtractIndices<PointT> extract;

        // Set extractor input
        extract.setInputCloud(inputCloud);
        extract.setIndices(inliers);

        // Extract inliers cloud
        extract.setNegative(false);
        extract.filter(*inliersCloud);

        // Extract outliers cloud (negative of inliers)
        extract.setNegative(true);
        extract.filter(*outliersCloud);
    }


    void planeSegmentation(
        typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
        typename pcl::PointCloud<PointT>::Ptr planeCloud,
        typename pcl::PointCloud<PointT>::Ptr objectsCloud,
        pcl::ModelCoefficients::Ptr coefficients,
        int maxIterations,
        float distanceThreshold)
    {
        auto startTime = std::chrono::steady_clock::now();

        pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
        pcl::SACSegmentation<PointT> seg;

        // Prepare segmentation object
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        // Do segmentation
        seg.setInputCloud(inputCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            // Plane not detected
            ROS_WARN_STREAM("Could not estimate plane for given dataset.");
            planeCloud->clear();
            *objectsCloud = *inputCloud;
        }
        else
        {
            // Plane detected - extract inliers and outliers into clouds
            extractInliersOutliers(inputCloud,
                                   planeCloud,
                                   objectsCloud,
                                   inliers);
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime =
            std::chrono::duration_cast<std::chrono::milliseconds>
            (endTime - startTime);
        ROS_INFO_STREAM("Plane segmentation took " << elapsedTime.count()
                        << " milliseconds");
    }


    std::vector<Object<PointT>> clustering(
        typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
        float clusterTolerance,
        int minClusterSize,
        int maxClusterSize)
    {
        auto startTime = std::chrono::steady_clock::now();

        std::vector<Object<PointT>> clusters;

        typename pcl::search::KdTree<PointT>::Ptr
            tree (new pcl::search::KdTree<PointT>);
        pcl::EuclideanClusterExtraction<PointT> ec;
        std::vector<pcl::PointIndices> clusterIndices;

        tree->setInputCloud(inputCloud);
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minClusterSize);
        ec.setMaxClusterSize(maxClusterSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(inputCloud);
        ec.extract(clusterIndices);

        for (auto it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
        {
            typename pcl::PointCloud<PointT>::Ptr
                cluster (new pcl::PointCloud<PointT>);
            auto begin = it->indices.begin();
            auto end = it->indices.end();
            for (auto cit = begin; cit != end; ++cit)
            {
                cluster->points.push_back(inputCloud->points[*cit]);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(Object<PointT>(cluster));
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime =
            std::chrono::duration_cast<std::chrono::milliseconds>
            (endTime - startTime);
        ROS_INFO_STREAM("Clustering took " << elapsedTime.count()
                        << " milliseconds and found " << clusters.size()
                        << " clusters" << std::endl);

        return clusters;
    }


    void voxelGridFilter(typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                         typename pcl::PointCloud<PointT>::Ptr filteredCloud,
                         float filterResolution)
    {
        auto startTime = std::chrono::steady_clock::now();

        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(inputCloud);
        vg.setLeafSize(filterResolution, filterResolution, filterResolution);
        vg.filter(*filteredCloud);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime =
            std::chrono::duration_cast<std::chrono::milliseconds>
            (endTime - startTime);
        ROS_INFO_STREAM("VoxelGridFilter took " << elapsedTime.count()
                        << " milliseconds");
    }
};
