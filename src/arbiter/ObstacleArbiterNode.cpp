#include "agribot_obstacle_detection/arbiter/ObstacleArbiterNode.h"

ObstacleArbiterNode::ObstacleArbiterNode(ros::NodeHandle* _nh)
  : nh(_nh)
  , TFListener(TFBuffer)
  , status(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  , sonarStatus(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
  , visionStatus(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
{
  // Advertise latched obstacles detection publisher
  obstacleDetectionPub = nh->advertise<agribot_msgs::AgribotObstacleDetection>("/agribot/obstacle_detected", 1, true);

  // Publish initial detection status (WAITING_FOR_FIRST_MEASUREMENT)
  publishDetectionStatus();

  // Load parameters
  loadParameters();

  // Create one-shot timeout timer and leave it in stopped state
  createPredictedPathTimeoutTimer();
  stopPredictedPathTimeoutTimer();

  ROS_INFO_STREAM("Created predicted path timeout timer with " << predictedPathTimeout << "s...");

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

  // Iterate over sonars
  for (const auto& sonar : msg->sonars)
  {
    if (sonar.status == agribot_msgs::SonarStatus::OBSTACLE_DETECTED)
    {
      // Break if any obstacle is detected
      ++detectedCount;
      break;
    }
    else if (sonar.status == agribot_msgs::SonarStatus::WAITING_FOR_FIRST_MEASUREMENT)
    {
      // Following sonars may have OBSTACLE_DETECTED status - can't break
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
}

void ObstacleArbiterNode::visionDetectionCallback(const agribot_msgs::VisionDetection::ConstPtr& msg)
{
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
  // TODO(olek) Change this method so it only performs checks (and separate conversions into different methods called
  // either in visionDetectionCallback or in predictedPathCallback)

  if (predictedPathPolygons.empty())
  {
    // Predicted path missing
    ROS_WARN_STREAM("Predicted path empty. Skipping vision detection...");
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
    return;
  }

  // TODO(olek) - add timeout check for list of obstacles (an maybe additional flag - obstaclesReceived?)
  if (visionPolygons.empty())
  {
    // Predicted path missing
    ROS_DEBUG_STREAM("Obstacles vector empty. Returning NO_OBSTACLE status...");
    visionStatus = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;
    return;
  }

  ROS_INFO_STREAM("Checking collisions...");

  // Check collisions
  for (const auto& robotPolygon : predictedPathPolygons)
  {
    for (const auto& visionPolygon : visionPolygons)
    {
      if (SAT::intersect(robotPolygon, visionPolygon))
      {
        // Any of the polygons intersect - set status and return (no need to check another polygons)
        visionStatus = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;
        return;
      }
    }
  }

  // No intersections detected - set status
  visionStatus = agribot_msgs::AgribotObstacleDetection::NO_OBSTACLE;
}

void ObstacleArbiterNode::updatePredictedPathPolygons(const std::vector<geometry_msgs::Point>& predictedPathPoints)
{
  for (std::size_t i = 0; i < predictedPathPoints.size(); i++)
  {
    // Create robot polygon with angle between previous and current point
    if (i > 0)
    {
      geometry_msgs::Point previousPoint = predictedPathPoints[i - 1];
      geometry_msgs::Point currentPoint = predictedPathPoints[i];
      float angle = getAngleBetweenPoints(previousPoint, currentPoint);
      agribot_msgs::PolygonStamped robotPolygon = getRobotPolygon(currentPoint, angle);
      predictedPathPolygons.push_back(robotPolygon);
    }

    // Create robot polygon with angle between current and next point
    if (i < predictedPathPoints.size() - 1)
    {
      geometry_msgs::Point currentPoint = predictedPathPoints[i];
      geometry_msgs::Point nextPoint = predictedPathPoints[i + 1];
      float angle = getAngleBetweenPoints(currentPoint, nextPoint);
      agribot_msgs::PolygonStamped robotPolygon = getRobotPolygon(currentPoint, angle);
      predictedPathPolygons.push_back(robotPolygon);
    }
  }
}

void ObstacleArbiterNode::updateVisionPolygons(const std::vector<agribot_msgs::DetectedObject>& visionObjects)
{
  std::vector<agribot_msgs::PolygonStamped> polygons;

  for (const auto& object : visionObjects)
  {
    agribot_msgs::PolygonStamped polygon = getVisionObjectPolygon(object);
    polygons.push_back(polygon);
  }

  // Rewrite vector with object polygons
  visionPolygons = polygons;
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
  // TODO(olek) - better casting (taking projection on ground plane) (or is it ok right now?)
  // Probably i should consider angle w.r.t to the ground (as lidar is mounted at some angle w.r.t. ground)
  agribot_msgs::PolygonStamped polygon;

  double dx = detectedObject.box.max_point.x - detectedObject.box.min_point.x;
  double dy = detectedObject.box.max_point.y - detectedObject.box.min_point.y;
  double z = detectedObject.box.min_point.z;

  geometry_msgs::Point polygonVertex;
  polygonVertex.x = detectedObject.box.min_point.x;
  polygonVertex.y = detectedObject.box.min_point.y;
  polygonVertex.z = z;
  polygon.polygon.points.push_back(polygonVertex);

  polygonVertex.x = detectedObject.box.min_point.x;
  polygonVertex.y = detectedObject.box.min_point.y + dy;
  polygonVertex.z = z;
  polygon.polygon.points.push_back(polygonVertex);

  polygonVertex.x = detectedObject.box.min_point.x + dx;
  polygonVertex.y = detectedObject.box.min_point.y;
  polygonVertex.z = z;
  polygon.polygon.points.push_back(polygonVertex);

  polygonVertex.x = detectedObject.box.min_point.x + dx;
  polygonVertex.y = detectedObject.box.min_point.y + dy;
  polygonVertex.z = z;
  polygon.polygon.points.push_back(polygonVertex);

  polygon.header.frame_id = visionFrameId;
  polygon.header.stamp = ros::Time::now();

  // Lookup transform from visionFrameId to predictedPathFrameId
  geometry_msgs::TransformStamped transform =
      TFBuffer.lookupTransform(visionFrameId, predictedPathFrameId, ros::Time::now(), ros::Duration(0.2));
  agribot_msgs::PolygonStamped polygonTransformed = doTransform(polygon, transform);

  return polygonTransformed;
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
  // Predicted path timeout
  nh->param<float>("predicted_path_timeout", predictedPathTimeout, 1.0);

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

void ObstacleArbiterNode::run()
{
  ros::spin();
}
