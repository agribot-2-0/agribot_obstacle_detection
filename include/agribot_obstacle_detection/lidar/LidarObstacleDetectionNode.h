/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <rviz_visual_tools/rviz_visual_tools.h>
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

  // Rviz visualization
  rviz_visual_tools::RvizVisualToolsPtr visualTools;

  // Node parameters
  bool bROSVis;
  bool bPublishGroundPlaneMarker;

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

    // Initialize ROS visualization
    if (bROSVis)
    {
      initROSVisualization();
    }

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

    /* Visualization */
    // Publish ROS visualization
    if (bROSVis)
    {
      publishROSVisualization();
    }
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

  void publishROSVisualization()
  {
    // Clear previous markers
    visualTools->deleteAllMarkers();

    // Publish bounding boxes for detected objects
    publishBoundingBoxArrayMarker();

    if (bPublishGroundPlaneMarker)
    {
      publishGroundPlaneMarker();
    }

    visualTools->trigger();
  }

  void publishBoundingBoxArrayMarker()
  {
    BoundingBox<PointT> box;

    for (std::size_t i = 0; i < detectedObjects.size(); ++i)
    {
      box = detectedObjects[i].getBoundingBox();

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      Eigen::Vector3d minPoint(box.xMin, box.yMin, box.zMin);
      Eigen::Vector3d maxPoint(box.xMax, box.yMax, box.zMax);

      visualTools->publishWireframeCuboid(pose, minPoint, maxPoint, rviz_visual_tools::BLUE, "Obstacle");

      // Translation for text alignment
      pose.translation() = Eigen::Vector3d(box.xMax, box.yMax, box.zMax);

      uint32_t id = detectedObjects[i].getId();
      std::string stringId = "Obstacle_" + std::to_string(id);

      visualTools->publishText(pose, stringId, rviz_visual_tools::WHITE, rviz_visual_tools::TEXTSIZE, false);
    }
  }

  void publishGroundPlaneMarker()
  {
    // Publish ground plane
    double A = groundPlaneCoefficients->values[0];
    double B = groundPlaneCoefficients->values[1];
    double C = groundPlaneCoefficients->values[2];
    double D = groundPlaneCoefficients->values[3];

    Eigen::Vector3d n(A, B, C);
    double distance = D / n.norm();
    Eigen::Vector3d center = -distance * n.normalized();

    Eigen::Isometry3d pose;
    pose.translation() = center;

    Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
    pose.linear() = q.toRotationMatrix();

    double height = 0.001;
    double x_width = 15.0;
    double y_width = 15.0;
    visualTools->publishCuboid(pose, x_width, y_width, height, rviz_visual_tools::ORANGE);
  }

  void run()
  {
    ros::spin();
  }

  void loadParameters()
  {
    // Load node parameters
    nh->getParam("ros_vis", bROSVis);
    nh->getParam("publish_ground_plane_marker", bPublishGroundPlaneMarker);

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

  void initROSVisualization()
  {
    visualTools.reset(new rviz_visual_tools::RvizVisualTools(frameId, "vision_markers"));
  }
};
