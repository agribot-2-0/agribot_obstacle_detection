/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>

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

  void radiusOutlierRemoval(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                            typename pcl::PointCloud<PointT>::Ptr filteredCloud, int neighbours_n, float radius)
  {
    auto startTime = std::chrono::steady_clock::now();

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(inputCloud);
    ror.setMinNeighborsInRadius(neighbours_n);
    ror.setRadiusSearch(radius);
    ror.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("radiusOutlierRemoval took " << elapsedTime.count() << " milliseconds");
  }

  void statisticalOutlierRemoval(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                                 typename pcl::PointCloud<PointT>::Ptr filteredCloud, int neighbours_n,
                                 float stddev_mul)
  {
    auto startTime = std::chrono::steady_clock::now();

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(inputCloud);
    sor.setMeanK(neighbours_n);
    sor.setStddevMulThresh(stddev_mul);
    sor.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("statisticalOutlierRemoval took " << elapsedTime.count() << " milliseconds");
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
    ROS_INFO_STREAM("Plane segmentation took " << elapsedTime.count() << " milliseconds");
  }

  std::vector<Object<PointT>> euclideanClustering(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
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

    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(inputCloud);

    for (const auto& it : clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

      // TODO(olek) - improve it, as make_shared makes copy of an object
      extract_indices.setIndices(boost::make_shared<pcl::PointIndices>(it));
      // pcl::PointIndices::Ptr indices_ptr(&it);
      extract_indices.filter(*cluster);
      clusters.push_back(Object<PointT>(cluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("Euclidean clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size()
                                                 << " clusters");

    return clusters;
  }

  std::vector<Object<PointT>> regionGrowingClustering(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                                                      int normal_estimation_neighbours_n, int neighbours_n,
                                                      float smoothness_angle_threshold, float curvature_threshold,
                                                      int minClusterSize, int maxClusterSize)
  {
    auto startTime = std::chrono::steady_clock::now();

    std::vector<Object<PointT>> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    // Stroing intermediate results
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Prepare search method
    tree->setInputCloud(inputCloud);

    // Prepare normal estimator
    pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
    normalEstimation.setSearchMethod(tree);
    normalEstimation.setInputCloud(inputCloud);
    normalEstimation.setKSearch(normal_estimation_neighbours_n);
    // normalEstimation.setRadiusSearch(0.03);
    normalEstimation.compute(*normals);

    pcl::RegionGrowing<PointT, pcl::Normal> rgc;
    // ec.setClusterTolerance(clusterTolerance);
    rgc.setMinClusterSize(minClusterSize);
    rgc.setMaxClusterSize(maxClusterSize);
    rgc.setSearchMethod(tree);
    rgc.setNumberOfNeighbours(neighbours_n);
    rgc.setInputCloud(inputCloud);
    rgc.setInputNormals(normals);
    rgc.setSmoothnessThreshold(smoothness_angle_threshold / 180.0 * M_PI);
    rgc.setCurvatureThreshold(curvature_threshold);

    std::vector<pcl::PointIndices> clusterIndices;
    rgc.extract(clusterIndices);

    pcl::ExtractIndices<PointT> extract_indices;
    extract_indices.setInputCloud(inputCloud);

    for (const auto& it : clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

      extract_indices.setIndices(boost::make_shared<pcl::PointIndices>(it));
      extract_indices.filter(*cluster);
      clusters.push_back(Object<PointT>(cluster));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("Region growing clustering took " << elapsedTime.count() << " milliseconds and found "
                                                      << clusters.size() << " clusters");

    return clusters;
  }

  void voxelGridFilter(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                       typename pcl::PointCloud<PointT>::Ptr filteredCloud, float filterResolution,
                       float distanceFilterMinX, float distanceFilterMaxX)
  {
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;

    vg.setFilterFieldName("x");
    vg.setFilterLimits(distanceFilterMinX, distanceFilterMaxX);

    vg.setInputCloud(inputCloud);
    vg.setLeafSize(filterResolution, filterResolution, filterResolution);
    vg.filter(*filteredCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("VoxelGridFilter took " << elapsedTime.count() << " milliseconds");
  }

  void rewriteCloud(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                    typename pcl::PointCloud<PointT>::Ptr filteredCloud)
  {
    auto startTime = std::chrono::steady_clock::now();

    filteredCloud = boost::make_shared<pcl::PointCloud<PointT>>(*inputCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("rewriteCloud took " << elapsedTime.count() << " milliseconds");
  }

  void distanceFilter(const typename pcl::PointCloud<PointT>::ConstPtr inputCloud,
                      typename pcl::PointCloud<PointT>::Ptr filteredCloud, float distanceThreshold)
  {
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    // typename pcl::KdTreeFLANN<PointT>::Ptr tree(new pcl::KdTreeFLANN<PointT>);
    PointT searchPoint;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    tree->setInputCloud(inputCloud);
    searchPoint.x = 0.0;
    searchPoint.y = 0.0;
    searchPoint.z = 0.0;

    if (tree->radiusSearch(searchPoint, distanceThreshold, pointIdxRadiusSearch, pointRadiusSquaredDistance))
    {
      filteredCloud->clear();
      for (const auto index : pointIdxRadiusSearch)
      {
        filteredCloud->points.push_back(inputCloud->points[index]);
      }
      filteredCloud->width = filteredCloud->points.size();
      filteredCloud->height = 1;
      filteredCloud->is_dense = true;
    }
    else
    {
      // No points to remove
      *filteredCloud = *inputCloud;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("distanceRemovingFilter took " << elapsedTime.count() << " milliseconds");
  }
};
