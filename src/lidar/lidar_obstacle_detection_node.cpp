#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include "agribot_obstacle_detection/lidar/LidarObstacleDetectionNode.h"

typedef pcl::PointXYZ Point;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_obstacle_detection_node");
    ros::NodeHandle nh("~");
    LidarObstacleDetectionNode<Point> lodn(&nh);
    lodn.run();
}
