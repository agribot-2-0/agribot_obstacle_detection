/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/obstacles_arbiter/ObstaclesArbiterNode.h"

ObstaclesArbiterNode::ObstaclesArbiterNode(ros::NodeHandle* nh, const std::string& log_tag,
                                           std::unique_ptr<IPolygonCollisionDetection> collision_detection)
  : nh_{ nh }, tf_listener_{ tf_buffer_ }, log_tag_{ log_tag }, collision_detection_{ std::move(collision_detection) }
{
  detection_status_.status = detection_status_.UNKNOWN;
  objects_target_frame_id_ = "base_footprint";

  typename dynamic_reconfigure::Server<agribot_obstacle_detection::ObstaclesArbiterConfig>::CallbackType
      dynamic_reconfigure_function = boost::bind(&ObstaclesArbiterNode::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_function);

  left_row_plane_rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint", "left_row_plane_marker"));
  right_row_plane_rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint", "right_row_plane_marker"));

  // Initialize publishers and subscribers
  out_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("out_polygons", 1, true);
  path_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("path_polygons", 1, true);
  objects_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("objects_polygons", 1, true);
  detection_status_pub_ = nh->advertise<agribot_msgs::ObstacleDetectionStatus>("detection_status", 1, true);
  left_row_path_pub_ = nh->advertise<nav_msgs::Path>("left_row_path", 1, true);
  right_row_path_pub_ = nh->advertise<nav_msgs::Path>("right_row_path", 1, true);

  processing_time_pub_ = nh->advertise<std_msgs::Float64>("processing_time", 1, false);

  in_object_array_sub_ = nh->subscribe("in_objects", 1, &ObstaclesArbiterNode::object3DArrayCallback, this);
  row_object_array_sub_ = nh->subscribe("row_objects", 1, &ObstaclesArbiterNode::rowObjectsCallback, this);
  path_sub_ = nh->subscribe("path", 1, &ObstaclesArbiterNode::pathCallback, this);

  buildRobotPolygon();
}

void ObstaclesArbiterNode::buildRobotPolygon()
{
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = "base_footprint";
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.pose.orientation.w = 1.0;

  robot_polygon_ = Conversions::convertToObjectPolygon(robot_pose, robot_safety_zone_length_, robot_safety_zone_width_);
}

void ObstaclesArbiterNode::object3DArrayCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  try
  {
    timer.start();

    if (msg->objects.size() == 0)
    {
      objects_polygons_ = agribot_msgs::ObjectPolygonArray();
    }
    else
    {
      if (msg->objects[0].header.frame_id != objects_target_frame_id_)
      {
        objects_polygons_ = Conversions::convertToConvexHullPolygonArray(
            Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        objects_polygons_ = Conversions::convertToConvexHullPolygonArray(*msg);
      }
    }

    checkCollisions();

    timer.stop();
    sendProcessingTimeInfo(timer.get_final_elapsed_time_ms());
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM(log_tag_ << e.what());
  }
}

void ObstaclesArbiterNode::rowObjectsCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  try
  {
    if (msg->objects.size() == 0)
    {
      row_polygons_ = agribot_msgs::ObjectPolygonArray();
    }
    else
    {
      if (msg->objects[0].header.frame_id != objects_target_frame_id_)
      {
        row_polygons_ = Conversions::convertToConvexHullPolygonArray(
            Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_));
      }
      else
      {
        row_polygons_ = Conversions::convertToConvexHullPolygonArray(*msg);
      }
    }

    computeRowsPaths();
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM(log_tag_ << e.what());
  }
}

void ObstaclesArbiterNode::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  try
  {
    timer.start();

    if (msg->header.frame_id != objects_target_frame_id_)
    {
      path_polygons_ = Conversions::convertToObjectPolygonArray(
          Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_), min_path_safety_zone_length_,
          min_path_safety_zone_width_, max_path_safety_zone_length_, max_path_safety_zone_width_,
          path_safety_zone_horizon_);
    }
    else
    {
      path_polygons_ = Conversions::convertToObjectPolygonArray(
          *msg, min_path_safety_zone_length_, min_path_safety_zone_width_, max_path_safety_zone_length_,
          max_path_safety_zone_width_, path_safety_zone_horizon_);
    }

    checkCollisions();

    timer.stop();
    sendProcessingTimeInfo(timer.get_final_elapsed_time_ms());
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM(log_tag_ << e.what());
  }
}

void ObstaclesArbiterNode::checkCollisions()
{
  if (objects_polygons_ == std::nullopt)
  {
    detection_status_.status = detection_status_.WAITING_FOR_OBJECTS;
    publishDetectionStatus();

    ROS_WARN_STREAM_THROTTLE(5.0, log_tag_ << "Missing objects info. Skipping calculation of output polygons...");
    return;
  }

  if (path_polygons_ == std::nullopt)
  {
    detection_status_.status = detection_status_.WAITING_FOR_PATH;
    publishDetectionStatus();

    ROS_WARN_STREAM_THROTTLE(5.0, log_tag_ << "Missing path info. Checking collisions only for robot polygon...");
  }

  // Clear detection status before checking new collisions
  detection_status_.status = detection_status_.NO_OBSTACLE;
  robot_polygon_.type = robot_polygon_.PATH_CLEAR;

  // If there are no polygons in any of the lists, NO_OBSTACLE will be published as for loop will be omitted
  for (auto& object_polygon : objects_polygons_->polygons)
  {
    // Check collision with robot polygon
    if (collision_detection_->areColliding(object_polygon, robot_polygon_))
    {
      detection_status_.status = detection_status_.OBSTACLE_DETECTED;
      publishDetectionStatus();

      // Mark path and objects polygons with _COLLIDING type
      object_polygon.type = object_polygon.OBSTACLE_COLLIDING;
      robot_polygon_.type = object_polygon.PATH_COLLIDING;
    }

    // Check collisions with path polygons
    if (path_polygons_ != std::nullopt)
    {
      for (auto& path_polygon : path_polygons_->polygons)
      {
        if (collision_detection_->areColliding(object_polygon, path_polygon))
        {
          detection_status_.status = detection_status_.OBSTACLE_DETECTED;
          publishDetectionStatus();

          // Mark path and objects polygons with _COLLIDING type
          object_polygon.type = object_polygon.OBSTACLE_COLLIDING;
          path_polygon.type = object_polygon.PATH_COLLIDING;
        }
      }
    }
  }

  // Publish status after collision checking. Mainly to handle NO_OBSTACLE situation
  publishDetectionStatus();

  // Publish output polygons for visualization
  publishOutputPolygons();
}

void ObstaclesArbiterNode::computeRowsPaths()
{
  nav_msgs::Path left_row_path;
  nav_msgs::Path right_row_path;

  auto row_obstacle_on_the_left = [this](const agribot_msgs::ObjectPolygon& p) {
    return p.centroid.y > this->row_polygon_centroid_y_threshold_ && p.area > this->row_polygon_area_threshold_;
  };
  auto row_obstacle_on_the_right = [this](const agribot_msgs::ObjectPolygon& p) {
    return p.centroid.y < -this->row_polygon_centroid_y_threshold_ && p.area > this->row_polygon_area_threshold_;
  };

  if (std::none_of(row_polygons_->polygons.begin(), row_polygons_->polygons.end(), row_obstacle_on_the_left))
  {
    left_row_path.poses.clear();
    left_row_path.header.frame_id = "base_footprint";
    left_row_path.header.stamp = ros::Time::now();
    right_row_path.poses.clear();
    right_row_path.header.frame_id = "base_footprint";
    right_row_path.header.stamp = ros::Time::now();
    ROS_WARN_STREAM("[ObstaclesArbiterNode] No row obstacle on the left. Publishing empty row paths...");
  }
  else if (std::none_of(row_polygons_->polygons.begin(), row_polygons_->polygons.end(), row_obstacle_on_the_right))
  {
    left_row_path.poses.clear();
    left_row_path.header.frame_id = "base_footprint";
    left_row_path.header.stamp = ros::Time::now();
    right_row_path.poses.clear();
    right_row_path.header.frame_id = "base_footprint";
    right_row_path.header.stamp = ros::Time::now();
    ROS_WARN_STREAM("[ObstaclesArbiterNode] No row obstacle on the right. Publishing empty row paths...");
  }
  else
  {
    // Sort objects in descending order by area
    auto area_comparison = [](const agribot_msgs::ObjectPolygon& p1, const agribot_msgs::ObjectPolygon& p2) {
      return p1.area < p2.area;
    };
    std::sort(row_polygons_->polygons.rbegin(), row_polygons_->polygons.rend(), area_comparison);

    // Find biggest polygons on the left and right
    auto left_row_polygon_it =
        std::find_if(row_polygons_->polygons.begin(), row_polygons_->polygons.end(), row_obstacle_on_the_left);
    auto right_row_polygon_it =
        std::find_if(row_polygons_->polygons.begin(), row_polygons_->polygons.end(), row_obstacle_on_the_right);

    ROS_ERROR_STREAM("Left polygon area: " << left_row_polygon_it->area);
    ROS_ERROR_STREAM("Right polygon area: " << right_row_polygon_it->area);

    auto [left_line_a, left_line_b] =
        lineCoefficientsWithRowPlaneSegmentation(left_row_polygon_it->point_cloud, "left");
    left_row_path = pathFromLine(left_line_a, left_line_b, row_path_start_x_, row_path_end_x_);

    auto [right_line_a, right_line_b] =
        lineCoefficientsWithRowPlaneSegmentation(right_row_polygon_it->point_cloud, "right");
    right_row_path = pathFromLine(right_line_a, right_line_b, row_path_start_x_, row_path_end_x_);
  }

  left_row_path_pub_.publish(left_row_path);
  right_row_path_pub_.publish(right_row_path);
}

std::pair<double, double>
ObstaclesArbiterNode::lineCoefficientsWithRowLineSegmentation(const sensor_msgs::PointCloud2& pc2)
{
  pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(seg_method_type_);
  seg.setDistanceThreshold(seg_distance_threshold_);
  seg.setMaxIterations(seg_max_iterations_);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::ModelCoefficients line_coefficients;

  pcl_conversions::toPCL(pc2, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

  seg.setInputCloud(pcl_cloud);
  seg.segment(*inliers, line_coefficients);

  auto p1_x = line_coefficients.values[0];
  auto p1_y = line_coefficients.values[1];

  auto direction_x = line_coefficients.values[3];
  auto direction_y = line_coefficients.values[4];

  auto p2_x = p1_x + direction_x;
  auto p2_y = p1_y + direction_y;

  auto line_a = (p2_y - p1_y) / (p2_x - p1_x);
  auto line_b = p1_y - line_a * p1_x;

  return std::make_pair(line_a, line_b);
}

std::pair<double, double> ObstaclesArbiterNode::lineCoefficientsWithRowPlaneSegmentation(
    const sensor_msgs::PointCloud2& pc2, const std::string& row)
{
  pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::ModelCoefficients plane_coefficients;
  boost::shared_ptr<pcl::SACSegmentation<pcl::PointXYZ>> seg;
  boost::shared_ptr<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>> normal_seg;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  pcl_conversions::toPCL(pc2, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

  switch (seg_model_type_)
  {
    case agribot_obstacle_detection::ObstaclesArbiter_plane:
      seg.reset(new pcl::SACSegmentation<pcl::PointXYZ>);
      break;

    case agribot_obstacle_detection::ObstaclesArbiter_parallel_plane:
      seg.reset(new pcl::SACSegmentation<pcl::PointXYZ>);
      seg->setAxis(Eigen::Vector3f(parallel_axis_x_, parallel_axis_y_, parallel_axis_z_));
      seg->setEpsAngle(eps_angle_);
      break;

    case agribot_obstacle_detection::ObstaclesArbiter_normal_plane:
      normal_seg.reset(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);

      // Estimate point normals
      ne.setSearchMethod(tree);
      ne.setInputCloud(pcl_cloud);
      ne.setKSearch(seg_normal_neighbours_n_);
      ne.compute(*cloud_normals);

      normal_seg->setInputNormals(cloud_normals);
      normal_seg->setNormalDistanceWeight(seg_normal_distance_weight_);

      seg = normal_seg;

      break;

    case agribot_obstacle_detection::ObstaclesArbiter_normal_parallel_plane:
      normal_seg.reset(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);

      // Estimate point normals
      ne.setSearchMethod(tree);
      ne.setInputCloud(pcl_cloud);
      ne.setKSearch(seg_normal_neighbours_n_);
      ne.compute(*cloud_normals);

      normal_seg->setInputNormals(cloud_normals);
      normal_seg->setNormalDistanceWeight(seg_normal_distance_weight_);
      normal_seg->setAxis(Eigen::Vector3f(parallel_axis_x_, parallel_axis_y_, parallel_axis_z_));
      normal_seg->setEpsAngle(eps_angle_);

      seg = normal_seg;

      break;

    default:
      ROS_ERROR_STREAM("[ObstaclesArbiterNode] Unsupported segmentation model selected...");
      break;
  }

  seg->setOptimizeCoefficients(true);
  seg->setModelType(seg_model_type_);
  seg->setMethodType(seg_method_type_);
  seg->setDistanceThreshold(seg_distance_threshold_);
  seg->setMaxIterations(seg_max_iterations_);

  seg->setInputCloud(pcl_cloud);
  seg->segment(*inliers, plane_coefficients);

  auto row_plane_a = plane_coefficients.values[0];
  auto row_plane_b = plane_coefficients.values[1];
  auto row_plane_c = plane_coefficients.values[2];
  auto row_plane_d = plane_coefficients.values[3];

  if (row == "right")
  {
    right_row_plane_rviz_tools_->deleteAllMarkers();
    right_row_plane_rviz_tools_->publishABCDPlane(row_plane_a, row_plane_b, row_plane_c, row_plane_d,
                                                  rviz_visual_tools::RED, 15.0, 5.0);
    right_row_plane_rviz_tools_->trigger(false);
  }
  else if (row == "left")
  {
    left_row_plane_rviz_tools_->deleteAllMarkers();
    left_row_plane_rviz_tools_->publishABCDPlane(row_plane_a, row_plane_b, row_plane_c, row_plane_d,
                                                 rviz_visual_tools::GREEN, 15.0, 5.0);
    left_row_plane_rviz_tools_->trigger(false);
  }

  double cutoff_z = 0.77;

  double line_m = -row_plane_a / row_plane_b;
  double line_n = -(row_plane_c * cutoff_z + row_plane_d) / row_plane_b;

  return std::make_pair(line_m, line_n);
}

nav_msgs::Path ObstaclesArbiterNode::pathFromLine(double line_a, double line_b, double start_x, double end_x)
{
  nav_msgs::Path path;

  auto start_y = line_a * start_x + line_b;
  auto end_y = line_a * end_x + line_b;

  geometry_msgs::PoseStamped left_start_pose;
  left_start_pose.header = path.header;
  left_start_pose.pose.position.x = start_x;
  left_start_pose.pose.position.y = start_y;
  left_start_pose.pose.orientation.w = 1;

  geometry_msgs::PoseStamped left_end_pose;
  left_end_pose.header = path.header;
  left_end_pose.pose.position.x = end_x;
  left_end_pose.pose.position.y = end_y;
  left_end_pose.pose.orientation.w = 1;

  path.poses.push_back(left_start_pose);
  path.poses.push_back(left_end_pose);

  path.header.frame_id = "base_footprint";
  path.header.stamp = ros::Time::now();

  return path;
}

void ObstaclesArbiterNode::publishOutputPolygons()
{
  // Just rewrite path and obstacles polygons. They should have proper "type" set already
  output_polygons_.polygons.clear();

  std::size_t output_size = objects_polygons_->polygons.size() + 1;  // At least for objects and robot polygon

  if (path_polygons_ != std::nullopt)
  {
    output_size += path_polygons_->polygons.size();
  }

  output_polygons_.polygons.reserve(output_size);

  if (path_polygons_ != std::nullopt)
  {
    output_polygons_.polygons.insert(output_polygons_.polygons.end(), path_polygons_->polygons.begin(),
                                     path_polygons_->polygons.end());
  }

  output_polygons_.polygons.insert(output_polygons_.polygons.end(), objects_polygons_->polygons.begin(),
                                   objects_polygons_->polygons.end());
  output_polygons_.polygons.push_back(robot_polygon_);

  out_polygons_pub_.publish(output_polygons_);
}

void ObstaclesArbiterNode::publishDetectionStatus()
{
  detection_status_pub_.publish(detection_status_);
}

void ObstaclesArbiterNode::sendProcessingTimeInfo(double time_ms)
{
  if (publish_processing_time_)
  {
    std_msgs::Float64 processing_time_msg;
    processing_time_msg.data = time_ms;
    processing_time_pub_.publish(processing_time_msg);
  }

  if (print_processing_time_)
  {
    ROS_INFO_STREAM(log_tag_ << "Full processing took:\t" << std::setprecision(2) << time_ms << " [ms]");
  }
}

void ObstaclesArbiterNode::run() const
{
  ros::spin();
}

void ObstaclesArbiterNode::dynamicReconfigureCallback(
    const agribot_obstacle_detection::ObstaclesArbiterConfig& dynamic_config, uint32_t level)
{
  static_cast<void>(level);  // Suppress unused variable warning

  publish_processing_time_ = dynamic_config.publish_processing_time;
  print_processing_time_ = dynamic_config.print_processing_time;
  eps_angle_ = dynamic_config.eps_angle;
  parallel_axis_x_ = dynamic_config.parallel_axis_x;
  parallel_axis_y_ = dynamic_config.parallel_axis_y;
  parallel_axis_z_ = dynamic_config.parallel_axis_z;
  seg_distance_threshold_ = dynamic_config.seg_distance_threshold;
  seg_max_iterations_ = dynamic_config.seg_max_iterations;
  seg_model_type_ = dynamic_config.seg_model_type;
  seg_method_type_ = dynamic_config.seg_method_type;
  seg_normal_distance_weight_ = dynamic_config.seg_normal_distance_weight;
  seg_normal_neighbours_n_ = dynamic_config.seg_normal_neighbours_n;
  robot_safety_zone_length_ = dynamic_config.robot_safety_zone_length;
  robot_safety_zone_width_ = dynamic_config.robot_safety_zone_width;
  min_path_safety_zone_length_ = dynamic_config.min_path_safety_zone_length;
  min_path_safety_zone_width_ = dynamic_config.min_path_safety_zone_width;
  max_path_safety_zone_length_ = dynamic_config.max_path_safety_zone_length;
  max_path_safety_zone_width_ = dynamic_config.max_path_safety_zone_width;
  path_safety_zone_horizon_ = dynamic_config.path_safety_zone_horizon;
  row_path_start_x_ = dynamic_config.row_path_start_x;
  row_path_end_x_ = dynamic_config.row_path_end_x;
  row_polygon_centroid_y_threshold_ = dynamic_config.row_polygon_centroid_y_threshold;
  row_polygon_area_threshold_ = dynamic_config.row_polygon_area_threshold;
}
