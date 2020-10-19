/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>
#include <chrono>  // NOLINT [build/c++11]

#include "agribot_obstacle_detection/lidar/object.h"

template <typename PointT>
class CloudProcessor
{
public:
  CloudProcessor()
  {
  }

  ~CloudProcessor()
  {
  }

  void extractInliersOutliers(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                              typename pcl::PointCloud<PointT>::Ptr inliersCloud,
                              typename pcl::PointCloud<PointT>::Ptr outliersCloud, pcl::PointIndices::Ptr inliers)
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

  void planeSegmentation(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                         typename pcl::PointCloud<PointT>::Ptr planeCloud,
                         typename pcl::PointCloud<PointT>::Ptr objectsCloud, pcl::ModelCoefficients::Ptr coefficients,
                         int maxIterations, float distanceThreshold)
  {
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
    pcl::SACSegmentation<PointT> seg;

    // Prepare segmentation object
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);

    if (maxIterations != 0)
    {
      seg.setMaxIterations(maxIterations);
    }

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
      extractInliersOutliers(inputCloud, planeCloud, objectsCloud, inliers);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_DEBUG_STREAM("Plane segmentation took " << elapsedTime.count() << " milliseconds");
  }

  void planeSegmentationWithCutoff(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                                   typename pcl::PointCloud<PointT>::Ptr planeCloud,
                                   typename pcl::PointCloud<PointT>::Ptr objectsCloud,
                                   pcl::ModelCoefficients::Ptr coefficients, int maxIterations, float distanceThreshold)
  {
    auto startTime = std::chrono::steady_clock::now();

    // Apply passthrough filter to cutoff lower part of cloud before ssegmentation
    typename pcl::PointCloud<PointT>::Ptr cutoffCloud{ new pcl::PointCloud<PointT> };
    PointT minPoint, maxPoint;
    double minZ, maxZ, diffZ;

    // Compute minimal and maximal Z in pointcloud
    // Get only lower 10% of pointcloud, but at least 0.1m
    pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);
    minZ = minPoint.z;
    maxZ = maxPoint.z;

    diffZ = 0.1 * (maxZ - minZ);
    if (diffZ < 0.1)
    {
      diffZ = 0.1;
    }

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minZ, minZ + diffZ);
    pass.filter(*cutoffCloud);

    // Prepare segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };

    seg.setInputCloud(cutoffCloud);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);

    if (maxIterations != 0)
    {
      seg.setMaxIterations(maxIterations);
    }

    // Do segmentation
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
      // Plane detected - extract (from ORIGINAL inputCloud) inliers and outliers into plane and objects
      extractInliersOutliers(inputCloud, planeCloud, objectsCloud, inliers);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_DEBUG_STREAM("Plane segmentation took " << elapsedTime.count() << " milliseconds");
  }

  std::vector<Object<PointT>> clustering(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                                         float clusterTolerance, int minClusterSize, int maxClusterSize)
  {
    auto startTime = std::chrono::steady_clock::now();

    std::vector<Object<PointT>> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::EuclideanClusterExtraction<PointT> ec;
    std::vector<pcl::PointIndices> clusterIndices;

    tree->setInputCloud(inputCloud);
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(inputCloud);
    ec.extract(clusterIndices);

    for (const auto& it : clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

      for (const auto& index : it.indices)
      {
        cluster->points.push_back(inputCloud->points[index]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(Object<PointT>(cluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_DEBUG_STREAM("Clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
                                       << " clusters" << std::endl);

    return clusters;
  }

  void voxelGridFilter(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                       typename pcl::PointCloud<PointT>::Ptr filteredCloud, float filterResolution)
  {
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(inputCloud);
    vg.setLeafSize(filterResolution, filterResolution, filterResolution);
    vg.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_DEBUG_STREAM("VoxelGridFilter took " << elapsedTime.count() << " milliseconds");
  }
};
