/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/ObjectClass.h>
#include <agribot_msgs/DetectedObject.h>
#include <agribot_msgs/VisionDetection.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <string>

#include "agribot_obstacle_detection/lidar/cloud_processor.h"
#include "agribot_obstacle_detection/lidar/object.h"
#include "agribot_obstacle_detection/lidar/bounding_box.h"

template <typename PointT>
class LidarObstacleDetectionNode
{
private:
  // ROS handlers
  ros::NodeHandle* nh;
  ros::Subscriber pcSub;
  ros::Publisher visionDetectionPub;

  std::string frameId;

  // VoxelGrid filter parameters
  float voxelGridFilterResolution;

  // Segmentation parameters
  int segmentationMaxIterations;
  float segmentationDistanceThreshold;

  // Clustering parameters
  float clusteringClusterTolerance;
  int clusteringMinClusterSize;
  int clusteringMaxClusterSize;

  // Cloud processor
  CloudProcessor<PointT> processor;

  // Used clouds
  typename pcl::PointCloud<PointT>::Ptr filteredCloud;
  typename pcl::PointCloud<PointT>::Ptr planeCloud;
  typename pcl::PointCloud<PointT>::Ptr detectedObjectsCloud;

  // Plane coefficients
  pcl::ModelCoefficients::Ptr groundPlaneCoefficients;

  // Vector of detected objects
  std::vector<Object<PointT>> detectedObjects;

public:
  explicit LidarObstacleDetectionNode(ros::NodeHandle* _nh)
    : nh(_nh)
    , filteredCloud(new pcl::PointCloud<PointT>)
    , planeCloud(new pcl::PointCloud<PointT>)
    , detectedObjectsCloud(new pcl::PointCloud<PointT>)
    , groundPlaneCoefficients(new pcl::ModelCoefficients)
  {
    // Load detection & node parameters
    loadParameters();

    visionDetectionPub = nh->advertise<agribot_msgs::VisionDetection>("vision_detection", 1, true);

    pcSub =
        nh->subscribe<pcl::PointCloud<PointT>>("input_cloud", 1, &LidarObstacleDetectionNode::pointcloudCallback, this);
  }

  ~LidarObstacleDetectionNode()
  {
  }

  void pointcloudCallback(const typename pcl::PointCloud<PointT>::ConstPtr& msg)
  {
    /* Processing */
    // Process cloud
    processingPipeline(msg);

    // Compute bounding box and centroid of each cluster
    for (std::size_t i = 0; i < detectedObjects.size(); ++i)
    {
      detectedObjects[i].computeBoundingBox();
      detectedObjects[i].computeCentroid();

      // For now simply give unique index
      detectedObjects[i].setId(i);
    }

    // Convert PCL timestamp to ROS timestamp
    ros::Time stamp = pcl_conversions::fromPCL(msg->header.stamp);

    // Publish array with detected objects
    publishVisionDetection(stamp);
  }

  void processingPipeline(const typename pcl::PointCloud<PointT>::ConstPtr& inputCloud)
  {
    // VoxelGrid filtering
    processor.voxelGridFilter(inputCloud, filteredCloud, voxelGridFilterResolution);

    // Plane segmentation
    processor.planeSegmentation(filteredCloud, planeCloud, detectedObjectsCloud, groundPlaneCoefficients,
                                segmentationMaxIterations, segmentationDistanceThreshold);

    // Clustering
    detectedObjects = processor.clustering(detectedObjectsCloud, clusteringClusterTolerance, clusteringMinClusterSize,
                                           clusteringMaxClusterSize);
  }

  void publishVisionDetection(const ros::Time& stamp)
  {
    agribot_msgs::VisionDetection detection;

    for (const auto& it : detectedObjects)
    {
      // Fill in pose and bounding box information
      agribot_msgs::DetectedObject object;
      object = it.toROSMsg();

      // Set class to UNKNOWN
      agribot_msgs::ObjectClass objectClass;
      objectClass.object_class = agribot_msgs::ObjectClass::UNKNOWN;
      objectClass.probability = 1.0;
      // Add class to vector of classes
      object.classes.push_back(objectClass);

      // Add object to vector inside vision detection
      detection.objects.push_back(object);
    }

    detection.header.frame_id = frameId;
    detection.header.stamp = stamp;

    // Publish vision detection
    visionDetectionPub.publish(detection);
  }

  void run()
  {
    ros::spin();
  }

  void loadParameters()
  {
    // Load node parameters
    // Frame and topics parameters
    nh->getParam("lidar_frame_id", frameId);

    // Obstacle detection parameters
    nh->param<float>("voxel_grid_filter_resolution", voxelGridFilterResolution, 0.1);

    nh->param<int>("segmentation_max_iterations", segmentationMaxIterations, 1000);

    nh->param<float>("segmentation_distance_threshold", segmentationDistanceThreshold, 0.05);

    nh->param<float>("clustering_cluster_tolerance", clusteringClusterTolerance, 0.2);

    nh->param<int>("clustering_min_cluster_size", clusteringMinClusterSize, 10);

    nh->param<int>("clustering_max_cluster_size", clusteringMaxClusterSize, 1000);

    ROS_INFO_STREAM("[Lidar Obstacle Detection] Starting Lidar Obstacle Detection with parameters:\n\n"
                    << "### Voxel Grid Filter ###" << std::endl
                    << "- Resolution = " << voxelGridFilterResolution << "\n\n### Segmentation ###" << std::endl
                    << "- Max Iterations = " << segmentationMaxIterations << std::endl
                    << "- Distance Threshold = " << segmentationDistanceThreshold << "\n\n### Clustering ###"
                    << std::endl
                    << "- Cluster Tolerance = " << clusteringClusterTolerance << std::endl
                    << "- Min Cluster Size = " << clusteringMinClusterSize << std::endl
                    << "- Max Cluster Size = " << clusteringMaxClusterSize << std::endl);
  }
};
