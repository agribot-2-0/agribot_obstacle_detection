#include "agribot_obstacle_detection/arbiter/ObstacleArbiterNode.h"

ObstacleArbiterNode::ObstacleArbiterNode(ros::NodeHandle* _nh)
  : nh(_nh)
  ,
  // TFListener(*TFBuffer),
  status(agribot_msgs::AgribotObstacleDetection::WAITING_FOR_FIRST_MEASUREMENT)
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

  predictedPathSub = _nh->subscribe("/move_base/SimplePlannerROS/predicted_path", 1,
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
  visionObjects = msg->objects;

  visionDetectionPipeline();
}

void ObstacleArbiterNode::predictedPathCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  // Restart timeout timer
  restartPredictedPathTimeoutTimer();

  predictedPathFrameId = msg->header.frame_id;
  predictedPathPoints = msg->points;

  visionDetectionPipeline();
}

void ObstacleArbiterNode::visionDetectionPipeline()
{
  checkVisionCollisions();
  updateDetectionStatus();
  publishDetectionStatus();
}

void ObstacleArbiterNode::checkVisionCollisions()
{
  if (predictedPathPoints.empty())
  {
    // Predicted path missing
    visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
    return;
  }

  // TODO(olek) Transform obstacles to the polygons in the frame of path

  for (const auto& object : visionObjects)
  {
  }

  // Get current robot pose in predictedPathFrame
  geometry_msgs::PoseStamped robotPose = getRobotPoseInPredictedPathFrame();

  // Check collisions
  for (const auto& point : predictedPathPoints)
  {
    // Create robot polygon from previous and current point
    // TODO(olek) Get angle between points
    // TODO(olek) Use angle, robotPose and dimensions/offset to construct robot polygon

    Polygon previousCurrentRobotPolygon;

    // Check if robot polygon collides with any obstacle
    for (const auto& object : visionObjects)
    {
      // Create obstacle polygon
      // TODO(olek)

      // Check collision
      // TODO(olek)
    }

    // Create robot polygon from current and next point
    // TODO(olek)
    Polygon currentNextRobotPolygon;

    // Check if robot polygon collides with any obstacle
    for (const auto& object : visionObjects)
    {
      // Create obstacle polygon
      // TODO(olek)

      // Check collision
      // TODO(olek)
    }

    // visionStatus = agribot_msgs::AgribotObstacleDetection::OBSTACLE_DETECTED;

    // At least one obstacle detected - no need to check further
    // return;
  }
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
  predictedPathPoints.clear();
  visionStatus = agribot_msgs::AgribotObstacleDetection::WAITING_FOR_PREDICTED_PATH;
  updateDetectionStatus();
  publishDetectionStatus();
}

geometry_msgs::PoseStamped ObstacleArbiterNode::getRobotPoseInPredictedPathFrame()
{
  geometry_msgs::PoseStamped robotPoseInPredictedPathFrame;
  geometry_msgs::PoseStamped robotPoseInBaseFrame;

  /* robotPoseInBaseFrame has 0.0, 0.0, 0.0 position by default so by transforming it to
  // predictedPathFrame we obtain position of robot's center in predictedPathFrameId */
  robotPoseInBaseFrame.header.frame_id = "base_footprint";
  robotPoseInBaseFrame.header.stamp = ros::Time::now();

  TFBuffer->transform(robotPoseInBaseFrame, robotPoseInPredictedPathFrame, predictedPathFrameId, ros::Duration(0.2));
  return robotPoseInPredictedPathFrame;
}

Polygon ObstacleArbiterNode::detectedObjectToPolygon(const agribot_msgs::DetectedObject& detectedObject,
                                                     const std::string& targetFrameId) const
{
}

std::vector<Polygon> ObstacleArbiterNode::detectedObjectsToPolygons(
    const std::vector<agribot_msgs::DetectedObject>& detectedObjects, const std::string& targetFrameId) const
{
  std::vector<Polygon> polygons;

  for (const auto& object : detectedObjects)
  {
    Polygon polygon = detectedObjectToPolygon(object, targetFrameId);
    polygons.push_back(polygon);
  }

  return polygons;
}

void ObstacleArbiterNode::run()
{
  ros::spin();
}
