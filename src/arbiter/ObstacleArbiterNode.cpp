/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_obstacle_detection/arbiter/ObstacleArbiterNode.h"

ObstacleArbiterNode::ObstacleArbiterNode(ros::NodeHandle* _nh)
  : nh(_nh)
  , TFListener(TFBuffer)
  , status(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  , sonarStatus(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  , visionStatus(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  , visionMsgReceived(false)
{
  // Advertise latched obstacles detection publisher
  obstacleDetectionPub = nh->advertise<agribot_msgs::AgribotObstacleDetection>("/agribot/obstacle_detected", 1, true);

  // Publish initial detection status (WAITING_FOR_FIRST_MEASUREMENT)
  publishDetectionStatus();

  // Load parameters
  loadParameters();

  // Initialize rviz_visual_tools
  initVisualization();

  // Subscribers
  // TODO(olek) Change topic to /agribot/sonar_detection
  sonarDetectionSub = _nh->subscribe("/sonar_obstacle_detection_node/sonar_detection", 1,
                                     &ObstacleArbiterNode::sonarDetectionCallback, this);

  visionDetectionSub =
      _nh->subscribe("/agribot/vision_detection", 1, &ObstacleArbiterNode::visionDetectionCallback, this);

  pathSub = _nh->subscribe("/move_base/AgribotPlannerROS/local_path", 1, &ObstacleArbiterNode::pathCallback, this);
}

ObstacleArbiterNode::~ObstacleArbiterNode()
{
}

void ObstacleArbiterNode::sonarDetectionCallback(const agribot_msgs::SonarDetection::ConstPtr& msg)
{
  uint8_t detectedCount = 0;
  uint8_t waitingCount = 0;

  std::vector<agribot_msgs::SonarStatus> tempCollidingSonars;

  // Iterate over sonars
  for (const auto& sonar : msg->sonars)
  {
    if (sonar.status == agribot_msgs::SonarStatus::OBSTACLE_DETECTED)
    {
      ++detectedCount;
      tempCollidingSonars.push_back(sonar);
    }
    else if (sonar.status == agribot_msgs::SonarStatus::WAITING_FOR_FIRST_MEASUREMENT)
    {
      ++waitingCount;
    }
  }

  // Update sonarStatus
  if (detectedCount > 0)
  {
    sonarStatus = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;
  }
  else if (waitingCount > 0)
  {
    sonarStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
  }
  else
  {
    sonarStatus = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;
  }

  // Update general detection status and publish it
  updateDetectionStatus();
  publishDetectionStatus();

  // Rewrite temporary vector
  collidingSonars = tempCollidingSonars;

  // Update sonar detection visualization
  if (pathFrameId != "")
  {
    publishMarkers();
  }
  else
  {
    ROS_WARN_STREAM("[Obstacle Arbiter Node] pathFrameId not set yet. Skipping sonar detection "
                    "visualization...");
  }
}

void ObstacleArbiterNode::visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg)
{
  visionMsgReceived = true;

  visionFrameId = msg->header.frame_id;

  if (pathFrameId != "")
  {
    updateVisionPolygons(msg->objects);
    visionDetectionPipeline();
  }
  else
  {
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
    updateDetectionStatus();
    publishDetectionStatus();
    ROS_WARN_STREAM("[Obstacle Arbiter Node] pathFrameId not set yet. Skipping vision detection...");
  }
}

void ObstacleArbiterNode::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  pathFrameId = msg->header.frame_id;
  updateRobotPolygons(msg->poses);
  visionDetectionPipeline();
}

void ObstacleArbiterNode::visionDetectionPipeline()
{
  checkVisionCollisions();

  // Update general detection status and publish it
  updateDetectionStatus();
  publishDetectionStatus();

  // Publish markers
  publishMarkers();
}

void ObstacleArbiterNode::checkVisionCollisions()
{
  if (!visionMsgReceived)
  {
    // No vision message received (or timeout occured)
    ROS_WARN_STREAM("[Obstacle Arbiter Node] No vision message received yet. Skipping vision detection...");
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
    return;
  }

  if (pathPolygons.empty())
  {
    // Path missing
    ROS_WARN_STREAM("[Obstacle Arbiter Node] Path empty. Skipping vision detection...");
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
    return;
  }

  // Use temporary status to determine if status was changed in below loops
  uint8_t tempVisionStatus = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;

  // Check collisions
  for (auto& visionPolygonPair : visionPolygonsPairs)
  {
    for (const auto& robotPolygon : pathPolygons)
    {
      if (SAT::intersect(robotPolygon, visionPolygonPair.first))
      {
        // Update status
        tempVisionStatus = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;

        // Mark polygon as colliding, for visualization purposes
        visionPolygonPair.second = true;
      }
    }
  }

  // Rewrite temporary status to visionStatus
  visionStatus = tempVisionStatus;
}

void ObstacleArbiterNode::updateVisionToPathTransform()
{
  // Lookup transform from visionFrameId to pathFrameId
  try
  {
    visionToPathTransform = TFBuffer.lookupTransform(pathFrameId, visionFrameId, ros::Time::now(), ros::Duration(0.5));
  }
  catch (tf2::InvalidArgumentException& e)
  {
    ROS_ERROR_STREAM("[Obstacle Arbiter Node] Error occured: " << e.what());
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR_STREAM("[Obstacle Arbiter Node] Error occured: " << e.what());
  }
}

void ObstacleArbiterNode::updateRobotPolygons(const std::vector<geometry_msgs::PoseStamped>& pathPosesStamped)
{
  std::vector<agribot_msgs::PolygonStamped> polygons;

  for (const auto& poseStamped : pathPosesStamped)
  {
    agribot_msgs::PolygonStamped robotPolygon = getRobotPolygon(poseStamped.pose);
    polygons.push_back(robotPolygon);
  }

  // Rewrite vector with path polygons
  pathPolygons = polygons;
}

void ObstacleArbiterNode::updateVisionPolygons(const std::vector<agribot_msgs::DetectedObject>& visionObjects)
{
  std::vector<std::pair<agribot_msgs::PolygonStamped, bool>> polygons;

  updateVisionToPathTransform();

  for (const auto& object : visionObjects)
  {
    agribot_msgs::PolygonStamped polygon = getVisionPolygon(object);
    polygons.push_back(std::make_pair(polygon, false));
  }

  // Rewrite vector with object polygons
  visionPolygonsPairs = polygons;
}

agribot_msgs::PolygonStamped ObstacleArbiterNode::getRobotPolygon(geometry_msgs::Pose pose) const
{
  float a = robotWidth + widthOffset;
  float b = robotLength;

  agribot_msgs::PolygonStamped polygon;
  polygon.header.stamp = ros::Time::now();
  polygon.header.frame_id = "temp_map_1";

  // Create polygon based on size constraints
  geometry_msgs::Point polygonVertex;
  polygonVertex.x = b / 2;
  polygonVertex.y = a / 2;
  polygon.polygon.points.push_back(polygonVertex);

  polygonVertex.x = -b / 2;
  polygonVertex.y = a / 2;
  polygon.polygon.points.push_back(polygonVertex);

  polygonVertex.x = -b / 2;
  polygonVertex.y = -a / 2;
  polygon.polygon.points.push_back(polygonVertex);

  polygonVertex.x = b / 2;
  polygonVertex.y = -a / 2;
  polygon.polygon.points.push_back(polygonVertex);

  // Create rotation transform
  // TODO(olek) - maybe transforms can be combined
  geometry_msgs::TransformStamped rotationTransform;
  rotationTransform.header.frame_id = "temp_map_1";
  rotationTransform.child_frame_id = "temp_map_2";
  rotationTransform.header.stamp = ros::Time::now();
  rotationTransform.transform.rotation = pose.orientation;
  agribot_msgs::PolygonStamped rotatedPolygon = transformPolygon(polygon, rotationTransform);

  geometry_msgs::TransformStamped translationTransform;
  translationTransform.header.frame_id = "temp_map_2";
  translationTransform.child_frame_id = pathFrameId;
  translationTransform.header.stamp = ros::Time::now();
  translationTransform.transform.rotation.w = 1;
  translationTransform.transform.translation.x = pose.position.x;
  translationTransform.transform.translation.y = pose.position.y;
  agribot_msgs::PolygonStamped finalPolygon = transformPolygon(rotatedPolygon, translationTransform);

  // Return rotated polygon
  return finalPolygon;
}

agribot_msgs::PolygonStamped
ObstacleArbiterNode::getVisionPolygon(const agribot_msgs::DetectedObject& detectedObject) const
{
  agribot_msgs::PolygonStamped polygon;

  double dx = detectedObject.box.max_point.x - detectedObject.box.min_point.x;
  double dy = detectedObject.box.max_point.y - detectedObject.box.min_point.y;
  double z = 0.0;

  geometry_msgs::Point polygonVertex;
  geometry_msgs::Point polygonVertexTransformed;
  polygonVertex.x = detectedObject.box.min_point.x;
  polygonVertex.y = detectedObject.box.min_point.y;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygonVertex.x = detectedObject.box.min_point.x;
  polygonVertex.y = detectedObject.box.min_point.y + dy;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygonVertex.x = detectedObject.box.min_point.x + dx;
  polygonVertex.y = detectedObject.box.min_point.y + dy;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygonVertex.x = detectedObject.box.min_point.x + dx;
  polygonVertex.y = detectedObject.box.min_point.y;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygon.header.frame_id = pathFrameId;
  polygon.header.stamp = ros::Time::now();

  return polygon;
}

agribot_msgs::PolygonStamped ObstacleArbiterNode::transformPolygon(
    const agribot_msgs::PolygonStamped& inPolygon, const geometry_msgs::TransformStamped& transform) const
{
  agribot_msgs::PolygonStamped outPolygon;
  outPolygon.header.stamp = inPolygon.header.stamp;
  outPolygon.header.frame_id = transform.child_frame_id;

  for (const auto& point : inPolygon.polygon.points)
  {
    geometry_msgs::Point transformedPoint;
    tf2::doTransform(point, transformedPoint, transform);
    outPolygon.polygon.points.push_back(transformedPoint);
  }

  return outPolygon;
}

void ObstacleArbiterNode::updateDetectionStatus()
{
  if (sonarStatus == agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED ||
      visionStatus == agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED)
  {
    status = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;
  }
  else if (sonarStatus == agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT ||
           visionStatus == agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  {
    status = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
  }
  else if (visionStatus == agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH)
  {
    status = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
  }
  else
  {
    status = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;
  }
}

void ObstacleArbiterNode::publishDetectionStatus()
{
  agribot_msgs::AgribotObstacleDetection msg;
  msg.status = status;
  msg.sonar_status = sonarStatus;
  msg.vision_status = visionStatus;
  obstacleDetectionPub.publish(msg);
}

void ObstacleArbiterNode::loadParameters()
{
  // Geometrical detection parameters
  if (nh->hasParam("robot_length"))
  {
    nh->getParam("robot_length", robotLength);
  }
  else
  {
    ROS_FATAL_STREAM("[Obstacle Arbiter Node] Missing robot_length parameter. Exiting...");
    nh->shutdown();
    exit(0);
  }

  if (nh->hasParam("robot_width"))
  {
    nh->getParam("robot_width", robotWidth);
  }
  else
  {
    ROS_FATAL_STREAM("[Obstacle Arbiter Node] Missing robot_width parameter. Exiting...");
    nh->shutdown();
    exit(0);
  }

  if (nh->hasParam("width_offset"))
  {
    nh->getParam("width_offset", widthOffset);
  }
  else
  {
    ROS_FATAL_STREAM("[Obstacle Arbiter Node] Missing width_offset parameter. Exiting...");
    nh->shutdown();
    exit(0);
  }
}

void ObstacleArbiterNode::initVisualization()
{
  // TODO(olek) - change frameId and markerTopic (maybe ass class fields and params)
  std::string frameId = "map";
  std::string polygonsMarkersTopic = "/agribot/obstacle_polygons_markers";
  std::string sonarDetectionMarkersTopic = "/agribot/sonar_detection_markers";
  visualTools.reset(new rviz_visual_tools::RvizVisualTools(frameId, polygonsMarkersTopic, *nh));
  visualTools->loadMarkerPub(false, false);
}

void ObstacleArbiterNode::publishMarkers()
{
  visualTools->deleteAllMarkers();
  publishVisionPolygonsMarkers();
  publishRobotPolygonsMarkers();
  publishSonarDetectionMarkers();
  visualTools->trigger();
}

void ObstacleArbiterNode::publishVisionPolygonsMarkers()
{
  for (const auto& polygonPair : visionPolygonsPairs)
  {
    if (polygonPair.second == true)
    {
      // Polygon collides with robot polygon
      visualTools->publishPolygon(polygonPair.first.polygon, rviz_visual_tools::RED, rviz_visual_tools::XLARGE,
                                  "VisionPolygon");
    }
    else
    {
      // No collision
      visualTools->publishPolygon(polygonPair.first.polygon, rviz_visual_tools::GREEN, rviz_visual_tools::XLARGE,
                                  "VisionPolygon");
    }
  }
}

void ObstacleArbiterNode::publishRobotPolygonsMarkers()
{
  for (const auto& polygon : pathPolygons)
  {
    visualTools->publishPolygon(polygon.polygon, rviz_visual_tools::PURPLE, rviz_visual_tools::XLARGE, "RobotPolygon");
  }
}

void ObstacleArbiterNode::publishSonarDetectionMarkers()
{
  for (const auto& sonar : collidingSonars)
  {
    try
    {
      geometry_msgs::Pose conePoseInSonarFrame;
      geometry_msgs::Pose conePoseInPathFrame;

      // Lookup transform from sonar FrameId to pathFrameId
      geometry_msgs::TransformStamped sonarToPathTransform =
          TFBuffer.lookupTransform(pathFrameId, sonar.header.frame_id, ros::Time::now(), ros::Duration(0.5));

      // Transform cone pose from sonar frame to path frame
      conePoseInSonarFrame.orientation.w = 1.0;
      sonarToPathTransform.header.frame_id = sonar.header.frame_id;
      sonarToPathTransform.child_frame_id = pathFrameId;
      tf2::doTransform(conePoseInSonarFrame, conePoseInPathFrame, sonarToPathTransform);

      // Publish marker
      // TODO(olek) - change angle value
      visualTools->publishCone(conePoseInPathFrame, 1.0);
    }
    catch (tf2::InvalidArgumentException& e)
    {
      ROS_ERROR_STREAM("[Obstacle Arbiter Node] Error occured: " << e.what());
    }
    catch (ros::Exception& e)
    {
      ROS_ERROR_STREAM("[Obstacle Arbiter Node] Error occured: " << e.what());
    }
  }
}

void ObstacleArbiterNode::run()
{
  ros::spin();
}
