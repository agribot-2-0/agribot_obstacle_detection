#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>

#include "agribot_obstacle_detection/cloud_processor.h"
#include "agribot_obstacle_detection/object.h"
#include "agribot_obstacle_detection/bounding_box.h"

namespace pcl_vis = ::pcl::visualization;

// Point type selection
typedef pcl::PointXYZ Point;


class LidarObstacleDetectionNode
{
 private:
    // ROS handlers
    ros::NodeHandle *nh;
    ros::Subscriber pcSub;
    ros::Publisher bbArrayPub;

    std::string frameId;
    std::string pointcloudTopic;
    std::string markerTopic;

    // Rviz visualization
    rviz_visual_tools::RvizVisualToolsPtr visualTools;

    // PCL Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer;

    // Node parameters
    bool bPCLVis;
    bool bROSVis;

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
    CloudProcessor<Point> processor;

    // Used clouds
    pcl::PointCloud<Point>::Ptr inputCloud;
    pcl::PointCloud<Point>::Ptr filteredCloud;
    pcl::PointCloud<Point>::Ptr planeCloud;
    pcl::PointCloud<Point>::Ptr detectedObjectsCloud;

    // Plane coefficients
    pcl::ModelCoefficients::Ptr groundPlaneCoefficients;

    // Vector of detected objects
    std::vector<Object<Point>> detectedObjects;

 public:
    explicit LidarObstacleDetectionNode(ros::NodeHandle *_nh) :
        nh(_nh),
        inputCloud(new pcl::PointCloud<Point>),
        filteredCloud(new pcl::PointCloud<Point>),
        planeCloud(new pcl::PointCloud<Point>),
        detectedObjectsCloud(new pcl::PointCloud<Point>),
        groundPlaneCoefficients(new pcl::ModelCoefficients)
    {
        // Load detection & node parameters
        loadParameters();

        // Initialize PCL visualization
        if (bPCLVis)
        {
            initPCLVisualization();
        }

        // Initialize ROS visualization
        if (bROSVis)
        {
            initROSVisualization();
        }

        pcSub = nh->subscribe(pointcloudTopic,
                              1,
                              &LidarObstacleDetectionNode::pointcloudCallback,
                              this);
    }

    ~LidarObstacleDetectionNode()
    {
    }


    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // Convert PC2 to pcl::PointCloud
        pcl::fromROSMsg(*msg, *inputCloud);

        // Process cloud
        processingPipeline();

        /* Visualization */
        // Compute bounding boxes
        for (int i = 0; i < detectedObjects.size(); ++i)
        {
            detectedObjects[i].computeBoundingBox();
        }

        // Initialize PCL visualization
        if (bPCLVis)
        {
            PCLVisualization();
        }

        // Initialize ROS visualization
        if (bROSVis)
        {
            ROSVisualization();
        }
    }


    void processingPipeline()
    {
        // VoxelGrid filtering
        processor.voxelGridFilter(inputCloud,
                                  filteredCloud,
                                  voxelGridFilterResolution);

        // Plane segmentation
        processor.planeSegmentation(filteredCloud,
                                    planeCloud,
                                    detectedObjectsCloud,
                                    groundPlaneCoefficients,
                                    segmentationMaxIterations,
                                    segmentationDistanceThreshold);

        // Clustering
        detectedObjects = processor.clustering(detectedObjectsCloud,
                                               clusteringClusterTolerance,
                                               clusteringMinClusterSize,
                                               clusteringMaxClusterSize);
    }


    void PCLVisualization()
    {
        // Clear previous pointclouds and objects
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Plane visualization
        pcl_vis::PointCloudColorHandlerCustom<Point> planeCloudGreen(planeCloud,
                                                                 0, 255, 0);
        viewer->addPointCloud(planeCloud, planeCloudGreen, "planeCloud");

        // Input cloud visualization
        pcl_vis::PointCloudColorHandlerCustom<Point> inputCloudWhite(inputCloud,
                                                                 255, 255, 255);
        viewer->addPointCloud(inputCloud, inputCloudWhite, "inputCloud");

        // Clusters visualization - PCL viewer
        for (int i = 0; i < detectedObjects.size(); ++i)
        {
            detectedObjects[i].visualize(viewer, i);
        }
    }


    void ROSVisualization()
    {
        // Clear previous markers
        visualTools->deleteAllMarkers();

        // Publish bounding boxes for detected objects
        publishBoundingBoxArray();
        publishGroundPlane();
        visualTools->trigger();
    }


    void publishBoundingBoxArray()
    {
        BoundingBox<Point> box;

        for (size_t i = 0; i< detectedObjects.size(); ++i)
        {
            detectedObjects[i].computeBoundingBox();
            box = detectedObjects[i].getBoundingBox();

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            Eigen::Vector3d minPoint(box.xMin, box.yMin, box.zMin);
            Eigen::Vector3d maxPoint(box.xMax, box.yMax, box.zMax);

            visualTools->publishWireframeCuboid(pose, minPoint, maxPoint,
                rviz_visual_tools::RED , "Obstacle", i);

            // translation pose for text TODO
            pose.translation() = Eigen::Vector3d(box.xMax, box.yMax, box.zMax);

            visualTools->publishText(pose,
                                    "Obstacle_" + std::to_string(i),
                                    rviz_visual_tools::WHITE,
                                    rviz_visual_tools::TEXTSIZE,
                                    false);
        }
    }

    void publishGroundPlane()
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
        visualTools->publishCuboid(pose, x_width, y_width,
                                   height, rviz_visual_tools::ORANGE);
    }

    void run()
    {
        if (bPCLVis)
        {
            while (!viewer->wasStopped() && nh->ok())
            {
                ros::spinOnce();
                viewer->spinOnce();
            }
        }
        else
        {
            while (nh->ok())
            {
                ros::spinOnce();
            }
        }
    }

    void loadParameters()
    {
        // Node parameters
        nh->param<bool>("pcl_vis",
                bPCLVis,
                false);

        nh->param<bool>("ros_vis",
                bROSVis,
                true);


        // Frame and topics parameters
        nh->param<std::string>("lidar_frame_id",
                frameId,
                "lidar_link");

        nh->param<std::string>("pointcloud_topic",
                pointcloudTopic,
                "/agribot/lidar/pc2");

        nh->param<std::string>("marker_topic",
                markerTopic,
                "agribot_obstacles_markers");


        // Obstacle detection parameters
        nh->param<float>("voxel_grid_filter_resolution",
                         voxelGridFilterResolution,
                         0.1);

        nh->param<int>("segmentation_max_iterations",
                       segmentationMaxIterations,
                       1000);

        nh->param<float>("segmentation_distance_threshold",
                         segmentationDistanceThreshold,
                         0.05);

        nh->param<float>("clustering_cluster_tolerance",
                         clusteringClusterTolerance,
                         0.2);

        nh->param<int>("clustering_min_cluster_size",
                       clusteringMinClusterSize,
                       10);

        nh->param<int>("clustering_max_cluster_size",
                        clusteringMaxClusterSize,
                        1000);

        ROS_INFO_STREAM(
        "\nStarting Lidar Obstacle Detection with parameters:\n\n" <<
        "### Voxel Grid Filter ###" << std::endl <<
        "- Resolution = " << voxelGridFilterResolution <<
        "\n\n### Segmentation ###" << std::endl <<
        "- Max Iterations = " << segmentationMaxIterations << std::endl <<
        "- Distance Threshold = " << segmentationDistanceThreshold <<
        "\n\n### Clustering ###" << std::endl <<
        "- Cluster Tolerance = " << clusteringClusterTolerance << std::endl <<
        "- Min Cluster Size = " << clusteringMinClusterSize << std::endl <<
        "- Max Cluster Size = " << clusteringMaxClusterSize << std::endl);
    }

    void initPCLVisualization()
    {
        viewer.reset(new pcl_vis::PCLVisualizer("Lidar obstacle viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        double pos_x = -8.3, pos_y = 0.05, pos_z = 2.5;
        double up_x = 0.0369313, up_y = -0.0313284, up_z = 0.998827;
        viewer->setCameraPosition(pos_x, pos_y, pos_z, up_x, up_y, up_z);
    }

    void initROSVisualization()
    {
        visualTools.reset(
            new rviz_visual_tools::RvizVisualTools(frameId, markerTopic));
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection_node");
    ros::NodeHandle nh("~");

    LidarObstacleDetectionNode lodn(&nh);
    lodn.run();
}
