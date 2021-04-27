/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <std_msgs/Int64.h>
#include <agribot_msgs/ObjectClass.h>
#include <agribot_msgs/DetectedObject.h>
#include <agribot_msgs/VisionDetection.h>

#include <dynamic_reconfigure/server.h>
#include <agribot_obstacle_detection/LidarPointCloudPipelineConfig.h>

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

enum class ClusteringMethod : int
{
  // Must match values defined in clustering_method_enum in cfg/LidarPointCloudPipeline.cfg
  euclidean = 0,
  region_growing = 1
};

enum class OutlierRemovalMethod : int
{
  // Must match values defined in outlier_removal_method_enum in cfg/LidarPointCloudPipeline.cfg
  none = 0,
  radius = 1,
  statistical = 2
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
  dynamic_reconfigure::Server<agribot_obstacle_detection::LidarPointCloudPipelineConfig> dynamicReconfigureServer;
  dynamic_reconfigure::Server<agribot_obstacle_detection::LidarPointCloudPipelineConfig>::CallbackType
      dynamicReconfigureFunction;

  // Target frame id
  std::string cloudTargetFrameId;

  // VoxelGrid + distance filter parameters
  bool use_vgf_;
  float voxelGridFilterResolution;
  float distanceFilterMinX;
  float distanceFilterMaxX;

  // Outlier removal filtering
  OutlierRemovalMethod outlier_removal_method_;
  int ror_neighbours_n_;
  float ror_radius_;
  int sor_neighbours_n_;
  float sor_stddev_mul_;

  // Segmentation parameters
  int segmentationMaxIterations;
  float segmentationDistanceThreshold;

  // Clustering parameters
  ClusteringMethod clusteringMethod;
  int clusteringMinClusterSize;
  int clusteringMaxClusterSize;
  float ec_cluster_tolerance;
  float rgc_neighbours_n_;
  float rgc_smoothness_angle_threshold_;
  float rgc_curvature_threshold_;
  float rgc_normal_estimation_neighbours_n_;

  bool vis_publish_ground_cloud_;
  bool vis_publish_obstacles_cloud_;

  // Cloud processor
  CloudProcessor<PointT> processor;

  // Used clouds
  typename pcl::PointCloud<PointT>::Ptr transformedCloud;
  typename pcl::PointCloud<PointT>::Ptr downsampledCloud;
  typename pcl::PointCloud<PointT>::Ptr outliersRemovedCloud;
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
    , outliersRemovedCloud(new pcl::PointCloud<PointT>)
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
      if (vis_publish_ground_cloud_)
      {
        sensor_msgs::PointCloud2 groundMsg;
        pcl::toROSMsg(*planeCloud, groundMsg);
        groundPC2Pub.publish(groundMsg);
      }

      if (vis_publish_obstacles_cloud_)
      {
        sensor_msgs::PointCloud2 obstaclesMsg;
        pcl::toROSMsg(*obstaclesCloud, obstaclesMsg);
        obstaclesPC2Pub.publish(obstaclesMsg);
      }

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
    // VoxelGrid filter
    if (use_vgf_)
    {
      // Decimate points and cutoff too close or too far points to obtain pointcloud with smaller number of points
      processor.voxelGridFilter(inputCloud, downsampledCloud, voxelGridFilterResolution, distanceFilterMinX,
                                distanceFilterMaxX);
    }
    else
    {
      // Rewrite pointer if filter is disabled (not the most optimal way, but allows for easier testing)
      // And can't just copy ConstPtr to Ptr
      downsampledCloud = boost::make_shared<pcl::PointCloud<PointT>>(*inputCloud);
    }

    // Outlier removal
    switch (outlier_removal_method_)
    {
      case OutlierRemovalMethod::radius:
        processor.radiusOutlierRemoval(downsampledCloud, outliersRemovedCloud, ror_neighbours_n_, ror_radius_);
        break;

      case OutlierRemovalMethod::statistical:
        processor.statisticalOutlierRemoval(downsampledCloud, outliersRemovedCloud, sor_neighbours_n_, sor_stddev_mul_);
        break;

      case OutlierRemovalMethod::none:
      default:
        // Just rewrite cloud pointer
        outliersRemovedCloud = downsampledCloud;
        break;
    }

    // // Transform Pointcloud to base_footprint (to allow planeSegmentationWithCutoff extract lower part of pointcloud)
    // geometry_msgs::TransformStamped transform;
    // try
    // {
    //   ros::Time transformTime = pcl_conversions::fromPCL(inputCloud->header.stamp);

    //   transform =
    //       tfBuffer.lookupTransform(cloudTargetFrameId, inputCloud->header.frame_id, transformTime,
    //       ros::Duration(0.01));
    // }
    // catch (tf2::LookupException& e)
    // {
    //   ROS_WARN_STREAM("Processing pipeline omitted: " << e.what());
    //   return false;
    // }
    // catch (tf2::ExtrapolationException& e)
    // {
    //   ROS_WARN_STREAM("Processing pipeline omitted: " << e.what());
    //   return false;
    // }

    // pcl_ros::transformPointCloud(*outliersRemovedCloud, *transformedCloud, transform.transform);
    // transformedCloud->header.frame_id = cloudTargetFrameId;

    transformedCloud = outliersRemovedCloud;

    // Plane segmentation
    processor.planeSegmentation(transformedCloud, planeCloud, obstaclesCloud, groundPlaneCoefficients,
                                segmentationMaxIterations, segmentationDistanceThreshold);

    // Clustering
    switch (clusteringMethod)
    {
      case ClusteringMethod::euclidean:
        detectedObjects = processor.euclideanClustering(obstaclesCloud, ec_cluster_tolerance, clusteringMinClusterSize,
                                                        clusteringMaxClusterSize);
        break;

      case ClusteringMethod::region_growing:
        detectedObjects = processor.regionGrowingClustering(
            obstaclesCloud, rgc_normal_estimation_neighbours_n_, rgc_neighbours_n_, rgc_smoothness_angle_threshold_,
            rgc_curvature_threshold_, clusteringMinClusterSize, clusteringMaxClusterSize);
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

  void dynamicReconfigureCallback(const agribot_obstacle_detection::LidarPointCloudPipelineConfig& config, uint32_t level)
  {
    cloudTargetFrameId = config.cloud_target_frame_id;

    // VoxelGrid filter
    use_vgf_ = config.use_vgf;
    voxelGridFilterResolution = config.vgf_resolution;
    distanceFilterMinX = config.vgf_min_x;
    distanceFilterMaxX = config.vgf_max_x;

    outlier_removal_method_ = static_cast<OutlierRemovalMethod>(config.outlier_removal_method);
    ror_neighbours_n_ = config.ror_neighbours_n;
    ror_radius_ = config.ror_radius;
    sor_neighbours_n_ = config.sor_neighbours_n;
    sor_stddev_mul_ = config.sor_stddev_mul;

    segmentationMaxIterations = config.seg_iterations;
    segmentationDistanceThreshold = config.seg_distance_threshold;

    clusteringMethod = static_cast<ClusteringMethod>(config.clustering_method);
    clusteringMinClusterSize = config.clustering_min_cluster_size;
    clusteringMaxClusterSize = config.clustering_max_cluster_size;

    ec_cluster_tolerance = config.ec_cluster_tolerance;

    rgc_normal_estimation_neighbours_n_ = config.rgc_normal_estimation_neighbours_n;
    rgc_neighbours_n_ = config.rgc_neighbours_n;
    rgc_smoothness_angle_threshold_ = config.rgc_smoothness_angle_threshold;
    rgc_curvature_threshold_ = config.rgc_curvature_threshold;

    vis_publish_ground_cloud_ = config.vis_publish_ground_cloud;
    vis_publish_obstacles_cloud_ = config.vis_publish_obstacles_cloud;
  }
};
