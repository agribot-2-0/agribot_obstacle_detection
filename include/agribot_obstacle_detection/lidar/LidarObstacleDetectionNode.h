/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <std_msgs/Int64.h>
#include <agribot_msgs/ObjectClass.h>
#include <agribot_msgs/DetectedObject.h>
#include <agribot_msgs/VisionDetection.h>

#include <dynamic_reconfigure/server.h>
#include <agribot_obstacle_detection/LidarDetectionConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <vector>
#include <string>

#include "agribot_obstacle_detection/lidar/cloud_processor.h"
#include "agribot_obstacle_detection/lidar/object.h"
#include "agribot_obstacle_detection/lidar/bounding_box.h"

enum class ClusteringType : int
{
  // Must match values defined in cfg/LidarDetection.cfg
  euclidean = 0,
  region_growing = 1
};

template <typename PointT>
class LidarObstacleDetectionNode
{
private:
  // ROS handlers
  ros::NodeHandle* nh;
  ros::Subscriber pcSub;
  ros::Publisher visionDetectionPub;

  // Ground and obstacles pointclouds publishers
  ros::Publisher groundPC2Pub;
  ros::Publisher obstaclesPC2Pub;

  // Processing time publisher
  ros::Publisher processingTimePub;

  // TF Buffer and listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<agribot_obstacle_detection::LidarDetectionConfig> dynamicReconfigureServer;
  dynamic_reconfigure::Server<agribot_obstacle_detection::LidarDetectionConfig>::CallbackType
      dynamicReconfigureFunction;

  // Target frame id
  std::string cloudTargetFrameId;

  // VoxelGrid filter parameters
  float voxelGridFilterResolution;

  // Distance threshold filtering
  float distanceFilterMinX;
  float distanceFilterMaxX;

  // Segmentation parameters
  int segmentationMaxIterations;
  float segmentationDistanceThreshold;

  // Clustering parameters
  float clusteringClusterTolerance;
  int clusteringMinClusterSize;
  int clusteringMaxClusterSize;
  ClusteringType clusteringMethod;

  // Cloud processor
  CloudProcessor<PointT> processor;

  // Used clouds
  typename pcl::PointCloud<PointT>::Ptr transformedCloud;
  typename pcl::PointCloud<PointT>::Ptr shortRangeCloud;
  typename pcl::PointCloud<PointT>::Ptr downsampledCloud;
  typename pcl::PointCloud<PointT>::Ptr planeCloud;
  typename pcl::PointCloud<PointT>::Ptr obstaclesCloud;

  // Plane coefficients
  pcl::ModelCoefficients::Ptr groundPlaneCoefficients;

  // Vector of detected objects
  std::vector<Object<PointT>> detectedObjects;

public:
  explicit LidarObstacleDetectionNode(ros::NodeHandle* _nh)
    : nh(_nh)
    , tfListener(tfBuffer)
    , transformedCloud(new pcl::PointCloud<PointT>)
    , shortRangeCloud(new pcl::PointCloud<PointT>)
    , downsampledCloud(new pcl::PointCloud<PointT>)
    , planeCloud(new pcl::PointCloud<PointT>)
    , obstaclesCloud(new pcl::PointCloud<PointT>)
    , groundPlaneCoefficients(new pcl::ModelCoefficients)
  {
    // Set up dynamic reconfigure
    dynamicReconfigureFunction = boost::bind(&LidarObstacleDetectionNode::dynamicReconfigureCallback, this, _1, _2);
    dynamicReconfigureServer.setCallback(dynamicReconfigureFunction);

    visionDetectionPub = nh->advertise<agribot_msgs::VisionDetection>("vision_detection", 1, true);

    groundPC2Pub = nh->advertise<sensor_msgs::PointCloud2>("ground_cloud", 1, false);
    obstaclesPC2Pub = nh->advertise<sensor_msgs::PointCloud2>("obstacles_cloud", 1, false);

    processingTimePub = nh->advertise<std_msgs::Int64>("lidar_processing_time_ms", 1, false);

    pcSub =
        nh->subscribe<pcl::PointCloud<PointT>>("input_cloud", 1, &LidarObstacleDetectionNode::pointcloudCallback, this);
  }

  ~LidarObstacleDetectionNode()
  {
  }

  void pointcloudCallback(const typename pcl::PointCloud<PointT>::ConstPtr& msg)
  {
    // Measure processing time
    auto startTime = std::chrono::steady_clock::now();

    /* Processing */
    // Process cloud
    if (processingPipeline(msg))
    {
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

      // Publish ground and obstacles clouds
      sensor_msgs::PointCloud2 groundMsg;
      pcl::toROSMsg(*planeCloud, groundMsg);
      groundPC2Pub.publish(groundMsg);

      sensor_msgs::PointCloud2 obstaclesMsg;
      pcl::toROSMsg(*obstaclesCloud, obstaclesMsg);
      obstaclesPC2Pub.publish(obstaclesMsg);

      // Publish array with detected objects
      publishVisionDetection(stamp);
    }
    else
    {
      // Ignore failed processingPipeline - this may only occur at the system startup when no transform from
      // inputCloud->header.frame_id to cloudTargetFrameId is present in tfBuffer and shouldn't be a problem
      // WARNING: THIS NOTE IS ONLY VALID FOR PRELIMINARY VERSION OF THE SYSTEM!
    }

    // Inform about processing time
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    ROS_INFO_STREAM("pointcloudCallback took " << elapsedTime.count() << " milliseconds");
    ROS_INFO_STREAM("");
    std_msgs::Int64 proctime_msg;
    proctime_msg.data = elapsedTime.count();
    processingTimePub.publish(proctime_msg);
  }

  bool processingPipeline(const typename pcl::PointCloud<PointT>::ConstPtr& inputCloud)
  {
    // Decimate points and cutoff too close or too far points to obtain pointcloud with smaller number of points
    processor.voxelGridFilter(inputCloud, downsampledCloud, voxelGridFilterResolution, distanceFilterMinX, distanceFilterMaxX);

    // Transform Pointcloud to base_footprint (to allow planeSegmentationWithCutoff extract lwoer part of pointcloud)
    geometry_msgs::TransformStamped transform;
    try
    {
      ros::Time transformTime = pcl_conversions::fromPCL(inputCloud->header.stamp);

      transform =
          tfBuffer.lookupTransform(cloudTargetFrameId, inputCloud->header.frame_id, transformTime, ros::Duration(0.01));
    }
    catch (tf2::LookupException& e)
    {
      ROS_WARN_STREAM("Processing pipeline omitted: " << e.what());
      return false;
    }
    catch (tf2::ExtrapolationException& e)
    {
      ROS_WARN_STREAM("Processing pipeline omitted: " << e.what());
      return false;
    }

    pcl_ros::transformPointCloud(*downsampledCloud, *transformedCloud, transform.transform);
    transformedCloud->header.frame_id = cloudTargetFrameId;

    // Plane segmentation
    processor.planeSegmentation(transformedCloud, planeCloud, obstaclesCloud, groundPlaneCoefficients,
                                segmentationMaxIterations, segmentationDistanceThreshold);

    // Clustering
    switch(clusteringMethod)
    {
      case ClusteringType::euclidean:
        detectedObjects = processor.euclideanClustering(obstaclesCloud, clusteringClusterTolerance, clusteringMinClusterSize,
                                                        clusteringMaxClusterSize);
      break;

      case ClusteringType::region_growing:
        detectedObjects = processor.regionGrowingClustering(obstaclesCloud, clusteringClusterTolerance, clusteringMinClusterSize,
                                                            clusteringMaxClusterSize);
      break;
    }

    return true;
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

    detection.header.frame_id = cloudTargetFrameId;
    detection.header.stamp = stamp;

    // Publish vision detection
    visionDetectionPub.publish(detection);
  }

  void run()
  {
    ros::spin();
  }

  void dynamicReconfigureCallback(const agribot_obstacle_detection::LidarDetectionConfig& config, uint32_t level)
  {
    cloudTargetFrameId = config.cloud_target_frame_id;

    distanceFilterMinX = config.distance_filter_min_x;
    distanceFilterMaxX = config.distance_filter_max_x;
    voxelGridFilterResolution = config.voxel_grid_filter_resolution;

    segmentationMaxIterations = config.segmentation_max_iterations;
    segmentationDistanceThreshold = config.segmentation_distance_threshold;

    clusteringMethod = static_cast<ClusteringType>(config.clustering_method);

    clusteringClusterTolerance = config.ec_cluster_tolerance;
    clusteringMinClusterSize = config.clustering_min_cluster_size;
    clusteringMaxClusterSize = config.clustering_max_cluster_size;

  }
};
