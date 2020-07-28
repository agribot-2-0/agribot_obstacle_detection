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

  // Create one-shot timeout timers and leave them in stopped state
  createPredictedPathTimeoutTimer();
  createVisionTimeoutTimer();
  stopPredictedPathTimeoutTimer();
  stopVisionTimeoutTimer();

  // Initialize rviz_visual_tools
  initVisualization();

  // Subscribers
  sonarDetectionSub = _nh->subscribe("/sonar_obstacle_detection_node/sonar_detection", 1,
                                     &ObstacleArbiterNode::sonarDetectionCallback, this);

  visionDetectionSub =
      _nh->subscribe("/agribot/vision_detection", 1, &ObstacleArbiterNode::visionDetectionCallback, this);

  predictedPathSub = _nh->subscribe("/move_base/AgribotPlannerROS/predicted_path", 1,
                                    &ObstacleArbiterNode::predictedPathCallback, this);
}

ObstacleArbiterNode::~ObstacleArbiterNode()
{
}

void ObstacleArbiterNode::sonarDetectionCallback(const agribot_msgs::SonarDetection::ConstPtr& msg)
{
  uint8_t detectedCount = 0;
  uint8_t waitingCount = 0;

  std::vector<agribot_msgs::SonarStatus> collidingSonars;

  // Iterate over sonars
  for (const auto& sonar : msg->sonars)
  {
    if (sonar.status == agribot_msgs::SonarStatus::OBSTACLE_DETECTED)
    {
      ++detectedCount;
      collidingSonars.push_back(sonar);
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

  // Update sonar detection visualization
  if (predictedPathFrameId != "")
  {
    publishSonarDetectionMarkers(collidingSonars);
  }
  else
  {
    ROS_WARN_STREAM("predictedPathFrameId not set yet. Skipping sonar detection visualization...");
  }
}

void ObstacleArbiterNode::visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg)
{
  restartVisionTimeoutTimer();
  visionMsgReceived = true;

  visionFrameId = msg->header.frame_id;

  if (predictedPathFrameId != "")
  {
    updateVisionPolygons(msg->objects);
    visionDetectionPipeline();
  }
  else
  {
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
    updateDetectionStatus();
    publishDetectionStatus();
    ROS_WARN_STREAM("predictedPathFrameId not set yet. Skipping vision detection...");
  }
}

void ObstacleArbiterNode::predictedPathCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  // Restart timeout timer
  restartPredictedPathTimeoutTimer();

  predictedPathFrameId = msg->header.frame_id;
  updatePredictedPathPolygons(msg->points);
  visionDetectionPipeline();
}

void ObstacleArbiterNode::visionDetectionPipeline()
{
  checkVisionCollisions();

  // Update general detection status and publish it
  updateDetectionStatus();
  publishDetectionStatus();
}

void ObstacleArbiterNode::checkVisionCollisions()
{
  publishPolygonsMarkers();

  if (!visionMsgReceived)
  {
    // No vision message received (or timeout occured)
    ROS_WARN_STREAM("No vision message received yet. Skipping vision detection...");
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
    return;
  }

  if (predictedPathPolygons.empty())
  {
    // Predicted path missing
    ROS_WARN_STREAM("Predicted path empty. Skipping vision detection...");
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
    return;
  }

  // Use temporary status to determine if status was changed in below loops
  uint8_t tempVisionStatus = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;

  // Check collisions
  for (auto& visionPolygonPair : visionPolygonsPairs)
  {
    for (const auto& robotPolygon : predictedPathPolygons)
    {
      if (SAT::intersect(robotPolygon, visionPolygonPair.first))
      {
        tempVisionStatus = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;
        visionPolygonPair.second = true;
      }
    }
  }

  // Rewrite temporary status to visionStatus
  visionStatus = tempVisionStatus;
}

void ObstacleArbiterNode::updateVisionToPredictedPathTransform()
{
  // Lookup transform from visionFrameId to predictedPathFrameId
  try
  {
    visionToPredictedPathTransform =
        TFBuffer.lookupTransform(predictedPathFrameId, visionFrameId, ros::Time::now(), ros::Duration(0.2));
  }
  catch (tf2::InvalidArgumentException& e)
  {
    ROS_ERROR_STREAM("Error occured: " << e.what());
  }
  catch (ros::Exception& e)
  {
    ROS_ERROR_STREAM("Error occured: " << e.what());
  }
}

void ObstacleArbiterNode::updatePredictedPathPolygons(const std::vector<geometry_msgs::Point>& predictedPathPoints)
{
  std::vector<agribot_msgs::PolygonStamped> polygons;

  for (std::size_t i = 0; i < predictedPathPoints.size(); i++)
  {
    // Create robot polygon with angle between previous and current point
    if (i > 0)
    {
      geometry_msgs::Point previousPoint = predictedPathPoints[i - 1];
      geometry_msgs::Point currentPoint = predictedPathPoints[i];
      float angle = getAngleBetweenPoints(previousPoint, currentPoint);
      agribot_msgs::PolygonStamped robotPolygon = getRobotPolygon(currentPoint, angle);
      polygons.push_back(robotPolygon);
    }

    // Create robot polygon with angle between current and next point
    if (i < predictedPathPoints.size() - 1)
    {
      geometry_msgs::Point currentPoint = predictedPathPoints[i];
      geometry_msgs::Point nextPoint = predictedPathPoints[i + 1];
      float angle = getAngleBetweenPoints(currentPoint, nextPoint);
      agribot_msgs::PolygonStamped robotPolygon = getRobotPolygon(currentPoint, angle);
      polygons.push_back(robotPolygon);
    }
  }

  // Rewrite vector with path polygons
  predictedPathPolygons = polygons;
}

void ObstacleArbiterNode::updateVisionPolygons(const std::vector<agribot_msgs::DetectedObject>& visionObjects)
{
  std::vector<std::pair<agribot_msgs::PolygonStamped, bool>> polygons;

  updateVisionToPredictedPathTransform();

  for (const auto& object : visionObjects)
  {
    agribot_msgs::PolygonStamped polygon = getVisionObjectPolygon(object);
    polygons.push_back(std::make_pair(polygon, false));
  }

  // Rewrite vector with object polygons
  visionPolygonsPairs = polygons;
}

agribot_msgs::PolygonStamped ObstacleArbiterNode::getRobotPolygon(geometry_msgs::Point centerPoint, float angle) const
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
  tf2::Quaternion rotationQuaternion;
  rotationQuaternion.setRPY(0, 0, angle);
  tf2::convert(rotationQuaternion, rotationTransform.transform.rotation);
  agribot_msgs::PolygonStamped rotatedPolygon = doTransform(polygon, rotationTransform);

  geometry_msgs::TransformStamped translationTransform;
  translationTransform.header.frame_id = "temp_map_2";
  translationTransform.child_frame_id = predictedPathFrameId;
  translationTransform.header.stamp = ros::Time::now();
  translationTransform.transform.rotation.w = 1;
  translationTransform.transform.translation.x = centerPoint.x;
  translationTransform.transform.translation.y = centerPoint.y;
  agribot_msgs::PolygonStamped finalPolygon = doTransform(rotatedPolygon, translationTransform);

  // Return rotated polygon
  return finalPolygon;
}

agribot_msgs::PolygonStamped
ObstacleArbiterNode::getVisionObjectPolygon(const agribot_msgs::DetectedObject& detectedObject) const
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
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPredictedPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygonVertex.x = detectedObject.box.min_point.x;
  polygonVertex.y = detectedObject.box.min_point.y + dy;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPredictedPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygonVertex.x = detectedObject.box.min_point.x + dx;
  polygonVertex.y = detectedObject.box.min_point.y + dy;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPredictedPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygonVertex.x = detectedObject.box.min_point.x + dx;
  polygonVertex.y = detectedObject.box.min_point.y;
  polygonVertex.z = detectedObject.box.min_point.z;
  tf2::doTransform(polygonVertex, polygonVertexTransformed, visionToPredictedPathTransform);
  polygonVertexTransformed.z = z;
  polygon.polygon.points.push_back(polygonVertexTransformed);

  polygon.header.frame_id = predictedPathFrameId;
  polygon.header.stamp = ros::Time::now();

  return polygon;
}

float ObstacleArbiterNode::getAngleBetweenPoints(geometry_msgs::Point firstPoint,
                                                 geometry_msgs::Point secondPoint) const
{
  float dx = secondPoint.x - firstPoint.x;
  float dy = secondPoint.y - firstPoint.y;
  return std::atan2(dy, dx);
}

agribot_msgs::PolygonStamped ObstacleArbiterNode::doTransform(const agribot_msgs::PolygonStamped& inPolygon,
                                                              const geometry_msgs::TransformStamped& transform) const
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
  else if (visionStatus == agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  {
    status = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
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
  // TODO(olek) - change default timeouts (to 1.0?)
  // Predicted path timeout
  nh->param<float>("predicted_path_timeout", predictedPathTimeout, 3.0);

  // Vision timeout
  nh->param<float>("vision_timeout", visionTimeout, 3.0);

  // Geometrical detection parameters
  if (nh->hasParam("robot_length"))
  {
    nh->getParam("robot_length", robotLength);
  }
  else
  {
    ROS_FATAL_STREAM("Missing robot_length parameter. Exiting...");
    nh->shutdown();
    exit(0);
  }

  if (nh->hasParam("robot_width"))
  {
    nh->getParam("robot_width", robotWidth);
  }
  else
  {
    ROS_FATAL_STREAM("Missing robot_width parameter. Exiting...");
    nh->shutdown();
    exit(0);
  }

  if (nh->hasParam("width_offset"))
  {
    nh->getParam("width_offset", widthOffset);
  }
  else
  {
    ROS_FATAL_STREAM("Missing width_offset parameter. Exiting...");
    nh->shutdown();
    exit(0);
  }
}

void ObstacleArbiterNode::createPredictedPathTimeoutTimer()
{
  predictedPathTimeoutTimer = nh->createTimer(ros::Duration(predictedPathTimeout),
                                              &ObstacleArbiterNode::predictedPathTimeoutCallback, this, true);
}

void ObstacleArbiterNode::stopPredictedPathTimeoutTimer()
{
  predictedPathTimeoutTimer.stop();
}

void ObstacleArbiterNode::startPredictedPathTimeoutTimer()
{
  predictedPathTimeoutTimer.start();
}

void ObstacleArbiterNode::restartPredictedPathTimeoutTimer()
{
  predictedPathTimeoutTimer.stop();
  predictedPathTimeoutTimer.start();
}

void ObstacleArbiterNode::predictedPathTimeoutCallback(const ros::TimerEvent& event)
{
  ROS_ERROR_STREAM("Predicted path timeout exceeded " << predictedPathTimeout << "s...");

  // Clear predicted path and propagate status
  predictedPathPolygons.clear();
  visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
  updateDetectionStatus();
  publishDetectionStatus();
}

void ObstacleArbiterNode::createVisionTimeoutTimer()
{
  visionTimeoutTimer =
      nh->createTimer(ros::Duration(visionTimeout), &ObstacleArbiterNode::visionTimeoutCallback, this, true);
}

void ObstacleArbiterNode::stopVisionTimeoutTimer()
{
  visionTimeoutTimer.stop();
}

void ObstacleArbiterNode::startVisionTimeoutTimer()
{
  visionTimeoutTimer.start();
}

void ObstacleArbiterNode::restartVisionTimeoutTimer()
{
  visionTimeoutTimer.stop();
  visionTimeoutTimer.start();
}

void ObstacleArbiterNode::visionTimeoutCallback(const ros::TimerEvent& event)
{
  ROS_ERROR_STREAM("Vision timeout exceeded " << visionTimeout << "s...");

  // Clear predicted path and propagate status
  visionPolygonsPairs.clear();
  visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT;
  visionMsgReceived = false;
  updateDetectionStatus();
  publishDetectionStatus();
}

void ObstacleArbiterNode::initVisualization()
{
  // TODO(olek) - change frameId and markerTopic (maybe ass class fields and params)
  std::string frameId = "map";
  std::string polygonsMarkersTopic = "/agribot/obstacle_polygons_markers";
  std::string sonarDetectionMarkersTopic = "/agribot/sonar_detection_markers";
  polygonsVisualTools.reset(new rviz_visual_tools::RvizVisualTools(frameId, polygonsMarkersTopic));
  sonarDetectionVisualTools.reset(new rviz_visual_tools::RvizVisualTools(frameId, sonarDetectionMarkersTopic));
}

void ObstacleArbiterNode::publishPolygonsMarkers()
{
  // Clear previous markers
  clearPolygonsMarkers();

  // Publish vision objects and projected robot polygons
  publishVisionPolygonsMarkers();
  publishRobotPolygonsMarkers();
  polygonsVisualTools->trigger();
}

void ObstacleArbiterNode::clearPolygonsMarkers()
{
  polygonsVisualTools->deleteAllMarkers();
}

void ObstacleArbiterNode::publishVisionPolygonsMarkers()
{
  for (const auto& polygonPair : visionPolygonsPairs)
  {
    if (polygonPair.second == true)
    {
      // Polygon collides with robot polygon
      polygonsVisualTools->publishPolygon(polygonPair.first.polygon, rviz_visual_tools::RED, rviz_visual_tools::XLARGE,
                                          "VisionPolygon");
    }
    else
    {
      // No collision
      polygonsVisualTools->publishPolygon(polygonPair.first.polygon, rviz_visual_tools::GREEN,
                                          rviz_visual_tools::XLARGE, "VisionPolygon");
    }
  }
}

void ObstacleArbiterNode::publishRobotPolygonsMarkers()
{
  for (const auto& polygon : predictedPathPolygons)
  {
    polygonsVisualTools->publishPolygon(polygon.polygon, rviz_visual_tools::PURPLE, rviz_visual_tools::XLARGE,
                                        "RobotPolygon");
  }
}

void ObstacleArbiterNode::publishSonarDetectionMarkers(const std::vector<agribot_msgs::SonarStatus>& collidingSonars)
{
  // Clear previous markers
  clearSonarDetectionMarkers();

  for (const auto& sonar : collidingSonars)
  {
    try
    {
      geometry_msgs::Pose conePoseInSonarFrame;
      geometry_msgs::Pose conePoseInPredictedPathFrame;

      conePoseInSonarFrame.orientation.w = 1.0;

      // Lookup transform from sonar FrameId to predictedPathFrameId
      geometry_msgs::TransformStamped sonarToPredictedPathTransform =
          TFBuffer.lookupTransform(predictedPathFrameId, sonar.header.frame_id, ros::Time::now(), ros::Duration(0.2));
      // Transform cone pose from soanr frame to predicted path frame
      sonarToPredictedPathTransform.header.frame_id = sonar.header.frame_id;
      sonarToPredictedPathTransform.child_frame_id = predictedPathFrameId;

      tf2::doTransform(conePoseInSonarFrame, conePoseInPredictedPathFrame, sonarToPredictedPathTransform);

      // Publish marker
      // TODO(olek) - change angle value
      sonarDetectionVisualTools->publishCone(conePoseInPredictedPathFrame, 1.0);
    }
    catch (tf2::InvalidArgumentException& e)
    {
      ROS_ERROR_STREAM("Error occured: " << e.what());
    }
    catch (ros::Exception& e)
    {
      ROS_ERROR_STREAM("Error occured: " << e.what());
    }
  }

  sonarDetectionVisualTools->trigger();
}

void ObstacleArbiterNode::clearSonarDetectionMarkers()
{
  sonarDetectionVisualTools->deleteAllMarkers();
}

void ObstacleArbiterNode::run()
{
  ros::spin();
}
