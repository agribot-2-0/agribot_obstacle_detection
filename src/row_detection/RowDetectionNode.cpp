/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/row_detection/RowDetectionNode.h"

RowDetectionNode::RowDetectionNode(ros::NodeHandle* nh, const std::string& log_tag)
  : nh_{ nh }, tf_listener_{ tf_buffer_ }, log_tag_{ log_tag }
{
  objects_target_frame_id_ = "base_footprint";

  typename dynamic_reconfigure::Server<agribot_obstacle_detection::RowDetectionConfig>::CallbackType
      dynamic_reconfigure_function = boost::bind(&RowDetectionNode::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_function);

  left_row_plane_rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint", "left_row_plane_marker"));
  right_row_plane_rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint", "right_row_plane_marker"));
  row_perimeter_polygon_rviz_tools_.reset(
      new rviz_visual_tools::RvizVisualTools("base_footprint", "row_perimeter_polygon_marker"));
  row_axis_line_rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("base_footprint", "row_axis_line_marker"));

  // Initialize publishers and subscribers
  left_row_path_pub_ = nh->advertise<nav_msgs::Path>("left_row_path", 1, true);
  right_row_path_pub_ = nh->advertise<nav_msgs::Path>("right_row_path", 1, true);
  row_polygons_pub_ = nh->advertise<agribot_msgs::ObjectPolygonArray>("row_polygons", 1, true);

  row_object_array_sub_ = nh->subscribe("row_objects", 1, &RowDetectionNode::rowObjectsCallback, this);
  row_info_sub_ = nh->subscribe("/agribot/navigation_manager/row", 1, &RowDetectionNode::rowInfoCallback, this);
}

void RowDetectionNode::rowObjectsCallback(const agribot_msgs::Object3DArray::ConstPtr& msg)
{
  try
  {
    if (msg->objects.size() == 0)
    {
      row_polygons_ = agribot_msgs::ObjectPolygonArray();
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr row_hull_pointcloud{ new pcl::PointCloud<pcl::PointXYZ> };
      row_hull_pointcloud->push_back(row_begin_point_1_);
      row_hull_pointcloud->push_back(row_begin_point_2_);
      row_hull_pointcloud->push_back(row_end_point_2_);
      row_hull_pointcloud->push_back(row_end_point_1_);

      if (msg->objects[0].header.frame_id != objects_target_frame_id_)
      {
        row_polygons_ = Conversions::convertToConvexHullPolygonArrayWithRowLimits(
            Conversions::transform(*msg, objects_target_frame_id_, tf_buffer_), row_hull_pointcloud);
      }
      else
      {
        row_polygons_ = Conversions::convertToConvexHullPolygonArrayWithRowLimits(*msg, row_hull_pointcloud);
      }
    }

    // TODO(olek) - debug only
    row_polygons_pub_.publish(row_polygons_);

    if (is_in_row_)
    {
      computeRowsPaths();
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(2.0, log_tag_ << "Robot not in row. Skipping calculation of rows paths...");
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM(log_tag_ << e.what());
  }
}

void RowDetectionNode::rowInfoCallback(const agribot_msgs::Row::ConstPtr& msg)
{
  is_in_row_ = msg->presence;

  if (msg->begin.header.frame_id != objects_target_frame_id_)
  {
    row_begin_pose_ = tf_buffer_.transform(msg->begin, objects_target_frame_id_, ros::Duration(0.1));
  }
  else
  {
    row_begin_pose_ = msg->begin;
  }

  if (msg->end.header.frame_id != objects_target_frame_id_)
  {
    row_end_pose_ = tf_buffer_.transform(msg->end, objects_target_frame_id_, ros::Duration(0.1));
  }
  else
  {
    row_end_pose_ = msg->end;
  }

  if (is_in_row_)
  {
    row_begin_line_ = Conversions::perpendicularLine(row_begin_pose_);
    row_end_line_ = Conversions::perpendicularLine(row_end_pose_);

    auto begin_yaw = tf::getYaw(row_begin_pose_.pose.orientation);
    auto end_yaw = tf::getYaw(row_end_pose_.pose.orientation);

    row_begin_point_1_.x = row_begin_pose_.pose.position.x - sin(begin_yaw) * row_width_ / 2.0;
    row_begin_point_2_.x = row_begin_pose_.pose.position.x + sin(begin_yaw) * row_width_ / 2.0;
    row_begin_point_1_.y = row_begin_line_.a_ * row_begin_point_1_.x + row_begin_line_.b_;
    row_begin_point_2_.y = row_begin_line_.a_ * row_begin_point_2_.x + row_begin_line_.b_;
    row_begin_point_1_.z = 0.0;
    row_begin_point_2_.z = 0.0;

    row_end_point_1_.x = row_end_pose_.pose.position.x - sin(end_yaw) * row_width_ / 2.0;
    row_end_point_2_.x = row_end_pose_.pose.position.x + sin(end_yaw) * row_width_ / 2.0;
    row_end_point_1_.y = row_end_line_.a_ * row_end_point_1_.x + row_end_line_.b_;
    row_end_point_2_.y = row_end_line_.a_ * row_end_point_2_.x + row_end_line_.b_;
    row_end_point_1_.z = 0.0;
    row_end_point_2_.z = 0.0;

    row_perimeter_polygon_rviz_tools_->deleteAllMarkers();
    agribot_msgs::ObjectPolygon polygon;
    polygon.header = row_begin_pose_.header;
    polygon.polygon.points.push_back(Conversions::convertToPoint32Msg(row_begin_point_1_));
    polygon.polygon.points.push_back(Conversions::convertToPoint32Msg(row_begin_point_2_));
    polygon.polygon.points.push_back(Conversions::convertToPoint32Msg(row_end_point_2_));
    polygon.polygon.points.push_back(Conversions::convertToPoint32Msg(row_end_point_1_));
    row_perimeter_polygon_rviz_tools_->publishPolygon(polygon, rviz_visual_tools::PINK, rviz_visual_tools::XLARGE,
                                                      "row_outline_polygon");
    row_perimeter_polygon_rviz_tools_->trigger(false);

    row_axis_line_ = Conversions::lineThroughPoses(row_begin_pose_, row_end_pose_);

    row_axis_line_rviz_tools_->deleteAllMarkers();
    row_axis_line_rviz_tools_->publishLine(row_begin_pose_.pose.position, row_end_pose_.pose.position,
                                           rviz_visual_tools::YELLOW, rviz_visual_tools::XXLARGE);
    row_axis_line_rviz_tools_->trigger(false);
  }
}

void RowDetectionNode::computeRowsPaths()
{
  nav_msgs::Path left_row_path;
  nav_msgs::Path right_row_path;

  auto row_obstacle_with_sufficient_area = [this](const agribot_msgs::ObjectPolygon& p) {
    return p.area > this->row_polygon_area_threshold_;
  };

  auto row_obstacle_on_the_left = [this](const agribot_msgs::ObjectPolygon& p) {
    // Calculate row axis y for x given by cluster centroid
    auto row_axis_y = this->row_axis_line_.a_ * p.centroid.x + this->row_axis_line_.b_;
    return p.centroid.y > row_axis_y;
  };
  auto row_obstacle_on_the_right = [this](const agribot_msgs::ObjectPolygon& p) {
    // Calculate row axis y for x given by cluster centroid
    auto row_axis_y = this->row_axis_line_.a_ * p.centroid.x + this->row_axis_line_.b_;
    return p.centroid.y < row_axis_y;
  };

  auto row_obstacle_on_the_left_with_sufficient_area =
      [this, row_obstacle_on_the_left, row_obstacle_with_sufficient_area](const agribot_msgs::ObjectPolygon& p) {
        return row_obstacle_on_the_left(p) && row_obstacle_with_sufficient_area(p);
      };

  auto row_obstacle_on_the_right_with_sufficient_area =
      [this, row_obstacle_on_the_right, row_obstacle_with_sufficient_area](const agribot_msgs::ObjectPolygon& p) {
        return row_obstacle_on_the_right(p) && row_obstacle_with_sufficient_area(p);
      };

  if (std::none_of(row_polygons_.polygons.begin(), row_polygons_.polygons.end(),
                   row_obstacle_on_the_left_with_sufficient_area))
  {
    left_row_path.poses.clear();
    left_row_path.header.frame_id = "base_footprint";
    left_row_path.header.stamp = ros::Time::now();
    right_row_path.poses.clear();
    right_row_path.header.frame_id = "base_footprint";
    right_row_path.header.stamp = ros::Time::now();
    ROS_DEBUG_STREAM("[RowDetectionNode] No row obstacle on the left. Publishing empty row paths...");
  }
  else if (std::none_of(row_polygons_.polygons.begin(), row_polygons_.polygons.end(),
                        row_obstacle_on_the_right_with_sufficient_area))
  {
    left_row_path.poses.clear();
    left_row_path.header.frame_id = "base_footprint";
    left_row_path.header.stamp = ros::Time::now();
    right_row_path.poses.clear();
    right_row_path.header.frame_id = "base_footprint";
    right_row_path.header.stamp = ros::Time::now();
    ROS_DEBUG_STREAM("[RowDetectionNode] No row obstacle on the right. Publishing empty row paths...");
  }
  else
  {
    // Sort objects in descending order by area
    auto area_comparison = [](const agribot_msgs::ObjectPolygon& p1, const agribot_msgs::ObjectPolygon& p2) {
      return p1.area < p2.area;
    };
    std::sort(row_polygons_.polygons.rbegin(), row_polygons_.polygons.rend(), area_comparison);

    // Find biggest polygons on the left and right
    auto left_row_polygon_it = std::find_if(row_polygons_.polygons.begin(), row_polygons_.polygons.end(),
                                            row_obstacle_on_the_left_with_sufficient_area);
    auto right_row_polygon_it = std::find_if(row_polygons_.polygons.begin(), row_polygons_.polygons.end(),
                                             row_obstacle_on_the_right_with_sufficient_area);

    auto left_row_plane_coefficients = rowPlaneSegmentation(left_row_polygon_it->point_cloud);
    auto right_row_plane_coefficients = rowPlaneSegmentation(right_row_polygon_it->point_cloud);

    publishPlaneMarker(left_row_plane_coefficients, "left");
    publishPlaneMarker(right_row_plane_coefficients, "right");

    auto left_row_line_coefficients = lineFromPlane(left_row_plane_coefficients, row_plane_cutoff_z_);
    auto right_row_line_coefficients = lineFromPlane(right_row_plane_coefficients, row_plane_cutoff_z_);

    left_row_path = pathFromLine(left_row_line_coefficients, row_path_start_x_, row_path_end_x_);
    right_row_path = pathFromLine(right_row_line_coefficients, row_path_start_x_, row_path_end_x_);
  }

  left_row_path_pub_.publish(left_row_path);
  right_row_path_pub_.publish(right_row_path);
}

PlaneCoefficients RowDetectionNode::rowPlaneSegmentation(const sensor_msgs::PointCloud2& pc2)
{
  pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::ModelCoefficients plane_coefficients;
  boost::shared_ptr<pcl::SACSegmentation<pcl::PointXYZ>> segmentation;
  boost::shared_ptr<pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>> normal_segmentation;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  pcl_conversions::toPCL(pc2, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

  switch (seg_model_type_)
  {
    case agribot_obstacle_detection::RowDetection_plane:
      segmentation.reset(new pcl::SACSegmentation<pcl::PointXYZ>);
      break;

    case agribot_obstacle_detection::RowDetection_parallel_plane:
      segmentation.reset(new pcl::SACSegmentation<pcl::PointXYZ>);
      segmentation->setAxis(Eigen::Vector3f(parallel_axis_x_, parallel_axis_y_, parallel_axis_z_));
      segmentation->setEpsAngle(eps_angle_);
      break;

    case agribot_obstacle_detection::RowDetection_normal_plane:
      normal_segmentation.reset(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);

      // Estimate point normals
      normal_estimation.setSearchMethod(tree);
      normal_estimation.setInputCloud(pcl_cloud);
      normal_estimation.setKSearch(seg_normal_neighbours_n_);
      normal_estimation.compute(*cloud_normals);

      normal_segmentation->setInputNormals(cloud_normals);
      normal_segmentation->setNormalDistanceWeight(seg_normal_distance_weight_);

      segmentation = normal_segmentation;

      break;

    case agribot_obstacle_detection::RowDetection_normal_parallel_plane:
      normal_segmentation.reset(new pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal>);

      // Estimate point normals
      normal_estimation.setSearchMethod(tree);
      normal_estimation.setInputCloud(pcl_cloud);
      normal_estimation.setKSearch(seg_normal_neighbours_n_);
      normal_estimation.compute(*cloud_normals);

      normal_segmentation->setInputNormals(cloud_normals);
      normal_segmentation->setNormalDistanceWeight(seg_normal_distance_weight_);
      normal_segmentation->setAxis(Eigen::Vector3f(parallel_axis_x_, parallel_axis_y_, parallel_axis_z_));
      normal_segmentation->setEpsAngle(eps_angle_);

      segmentation = normal_segmentation;

      break;

    default:
      ROS_ERROR_STREAM("[RowDetectionNode] Unsupported segmentation model selected...");
      break;
  }

  segmentation->setOptimizeCoefficients(true);
  segmentation->setDistanceThreshold(seg_distance_threshold_);
  segmentation->setMaxIterations(seg_max_iterations_);
  segmentation->setModelType(seg_model_type_);
  segmentation->setMethodType(seg_method_type_);

  segmentation->setInputCloud(pcl_cloud);
  segmentation->segment(*inliers, plane_coefficients);

  return { plane_coefficients.values[0], plane_coefficients.values[1], plane_coefficients.values[2],
           plane_coefficients.values[3] };
}

LineCoefficients RowDetectionNode::lineFromPlane(const PlaneCoefficients& plane, double cutoff_height) const
{
  double line_a = -plane.a_ / plane.b_;
  double line_b = -(plane.c_ * cutoff_height + plane.d_) / plane.b_;

  return { line_a, line_b };
}

nav_msgs::Path RowDetectionNode::pathFromLine(const LineCoefficients& line, double start_x, double end_x) const
{
  nav_msgs::Path path;

  auto start_y = line.a_ * start_x + line.b_;
  auto end_y = line.a_ * end_x + line.b_;

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

void RowDetectionNode::publishPlaneMarker(const PlaneCoefficients& plane, const std::string& row) const
{
  if (row == "right")
  {
    right_row_plane_rviz_tools_->deleteAllMarkers();
    right_row_plane_rviz_tools_->publishABCDPlane(plane.a_, plane.b_, plane.c_, plane.d_, rviz_visual_tools::RED, 15.0,
                                                  5.0);
    right_row_plane_rviz_tools_->trigger(false);
  }
  else if (row == "left")
  {
    left_row_plane_rviz_tools_->deleteAllMarkers();
    left_row_plane_rviz_tools_->publishABCDPlane(plane.a_, plane.b_, plane.c_, plane.d_, rviz_visual_tools::GREEN, 15.0,
                                                 5.0);
    left_row_plane_rviz_tools_->trigger(false);
  }
}

void RowDetectionNode::run() const
{
  ros::spin();
}

void RowDetectionNode::dynamicReconfigureCallback(const agribot_obstacle_detection::RowDetectionConfig& dynamic_config,
                                                  uint32_t level)
{
  static_cast<void>(level);  // Suppress unused variable warning

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
  row_path_start_x_ = dynamic_config.row_path_start_x;
  row_path_end_x_ = dynamic_config.row_path_end_x;
  row_polygon_centroid_y_threshold_ = dynamic_config.row_polygon_centroid_y_threshold;
  row_polygon_area_threshold_ = dynamic_config.row_polygon_area_threshold;
  row_plane_cutoff_z_ = dynamic_config.row_plane_cutoff_z;
  row_width_ = dynamic_config.row_width;
}
