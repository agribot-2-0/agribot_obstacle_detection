#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

#include "agribot_obstacle_detection/lidar/cloud_processor.h"
#include "agribot_obstacle_detection/lidar/object.h"
#include "agribot_obstacle_detection/lidar/bounding_box.h"


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

    // Node parameters
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
        filteredCloud(new pcl::PointCloud<Point>),
        planeCloud(new pcl::PointCloud<Point>),
        detectedObjectsCloud(new pcl::PointCloud<Point>),
        groundPlaneCoefficients(new pcl::ModelCoefficients)
    {
        pointcloudTopic = "input_cloud";
        markerTopic = "obstacle_markers";

        // Load detection & node parameters
        loadParameters();

        // Initialize ROS visualization
        if (bROSVis)
        {
            initROSVisualization();
        }

        // pcSub = nh->subscribe(pointcloudTopic,
        pcSub = nh->subscribe<pcl::PointCloud<Point>>(pointcloudTopic,
                              1,
                              &LidarObstacleDetectionNode::pointcloudCallback,
                              this);
    }

    ~LidarObstacleDetectionNode()
    {
    }

    void pointcloudCallback(const pcl::PointCloud<Point>::ConstPtr& msg)
    {
        // Process cloud
        processingPipeline(msg);

        /* Visualization */
        // Compute bounding boxes
        for (int i = 0; i < detectedObjects.size(); ++i)
        {
            detectedObjects[i].computeBoundingBox();
        }

        // Initialize ROS visualization
        if (bROSVis)
        {
            ROSVisualization();
        }
    }


    void processingPipeline(const pcl::PointCloud<Point>::ConstPtr& inputCloud)
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
        ros::spin();
    }

    void loadParameters()
    {
        nh->param<bool>("ros_vis",
                bROSVis,
                true);

        // Frame and topics parameters
        nh->param<std::string>("lidar_frame_id",
                frameId,
                "lidar_link");

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
