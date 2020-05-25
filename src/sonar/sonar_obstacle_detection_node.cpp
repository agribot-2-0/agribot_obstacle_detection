#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <agribot_msgs/SonarStatus.h>
#include <agribot_msgs/SonarDetection.h>

#include "agribot_obstacle_detection/sonar/sonar.h"


class SonarObstacleDetectionNode
{
 private:
    // ROS handlers
    ros::NodeHandle *nh;
    ros::Publisher sonarDetectionPub;

    // ROS variables
    ros::Time lastTimestamp;

    // Sonars map with id as a key
    std::map<std::string, Sonar> sonars;

    // Subscribers map with id as a key
    std::map<std::string, std::shared_ptr<ros::Subscriber>> subscribers;

    // SonarDetection message
    agribot_msgs::SonarDetection sonarDetectionMsg;

 public:
    explicit SonarObstacleDetectionNode(ros::NodeHandle *_nh) :
        nh(_nh),
        lastTimestamp(ros::Time::now())
    {
        // Create sonar detection publisher with latched connection
        sonarDetectionPub =
            nh->advertise<agribot_msgs::SonarDetection>("sonar_detection",
                                                        1,
                                                        true);

        // Load sonar parameters
        loadSonars();

        // Create initial SonarDetection message based on loaded sonars
        initSonarDetectionMsg();
        // Publish initial msg with WAITING_FOR_MEASUREMENT status
        publishSonarDetecionMsg();

        // Register sonar subscribers
        registerSubscribers();
    }

    ~SonarObstacleDetectionNode()
    {
    }

    void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,
                       std::string id)
    {
        // Add range to sonar detector object assigned to id
        auto sonar = sonars.find(id);
        sonar->second.addRange(msg->range);


        // Check if obstacle is present in any of measurements
        uint8_t status;
        if (sonar->second.isObstacleDetected())
        {
            status = agribot_msgs::SonarStatus::OBSTACLE_DETECTED;
        }
        else
        {
            status = agribot_msgs::SonarStatus::NO_OBSTACLE;
        }

        // Get shortest and last range of the sonar
        float shortestRange = sonar->second.getShortestRange();
        float lastRange = sonar->second.getLastRange();

        // Update lastTimestamp
        updateLastTimestamp(msg->header.stamp);

        // Update and publish SonarDetection message
        updateSonarDetectionMsg(id, status, shortestRange, lastRange);
        publishSonarDetecionMsg();
    }

    void updateSonarDetectionMsg(std::string id,
                                 uint8_t status,
                                 float shortestRange,
                                 float lastRange)
    {
        // Update status for sonar matching id
        auto begin = sonarDetectionMsg.sonars.begin();
        auto end = sonarDetectionMsg.sonars.end();

        for (auto it = begin; it != end; ++it)
        {
            if (it->id == id)
            {
                it->status = status;
                it->shortest_range = shortestRange;
                it->last_range = lastRange;
                break;
            }
        }
    }

    void publishSonarDetecionMsg()
    {
        sonarDetectionMsg.header.stamp = lastTimestamp;
        sonarDetectionPub.publish(sonarDetectionMsg);
    }

    void updateLastTimestamp(ros::Time newTimestamp)
    {
        ros::Duration timeDiff = newTimestamp - lastTimestamp;

        // Update only if newTimestamp is newer than lastTimestamp
        if (timeDiff.toNSec() > 0)
        {
            lastTimestamp = newTimestamp;
        }
    }

    void registerSubscribers()
    {
        for (auto it = sonars.begin(); it != sonars.end(); ++it)
        {
            std::string id = it->first;
            std::string topic = it->second.getTopic();

            // Create subscriber ptr to avoid copying, when inserting into map
            std::shared_ptr<ros::Subscriber> sub;
            sub = std::make_shared<ros::Subscriber>();

            // Create boost::bind callback function, which passes
            // appropriate id string, used to determine source
            // of incoming measurement
            auto boostSonarCallback = boost::bind(
                &SonarObstacleDetectionNode::sonarCallback,
                this,
                _1,
                id);

            // Subscribe with boostSonarCallback function
            *sub = nh->subscribe<sensor_msgs::Range>(
                topic,
                1,
                boostSonarCallback);

            // Add subscriber to subscribers map
            std::pair<std::string, std::shared_ptr<ros::Subscriber>> subPair(
                topic, sub);
            subscribers.insert(subPair);

            // Print information
            ROS_INFO_STREAM_NAMED("sonar_obstacle_detection_node",
                                  "Subscribed to topic: " << topic);
        }
    }

    void initSonarDetectionMsg()
    {
        // Add entry to sonarDetectionMsg for every laoded sonar
        for (auto it = sonars.begin(); it != sonars.end(); ++it)
        {
            agribot_msgs::SonarStatus sonarStatusMsg;
            sonarStatusMsg.id = it->second.getId();
            sonarStatusMsg.status =
                agribot_msgs::SonarStatus::WAITING_FOR_MEASUREMENT;
            sonarStatusMsg.threshold = it->second.getThreshold();
            sonarDetectionMsg.sonars.push_back(sonarStatusMsg);
        }
    }

    void loadSonars()
    {
        bool allParametersLoaded = false;
        std::size_t i = 0;

        std::string sonarField, thresholdField, topicField, historyLengthField;

        float threshold;
        int historyLength;
        std::string id, topic;

        // Check all configurations starting from sonar0
        while (!allParametersLoaded)
        {
            sonarField = "sonar" + std::to_string(i);
            topicField = sonarField + "_topic";
            thresholdField = sonarField + "_threshold";
            historyLengthField = sonarField + "_history_length";

            if (nh->hasParam(sonarField))
            {
                if (nh->hasParam(topicField)
                    && nh->hasParam(thresholdField)
                    && nh->hasParam(historyLengthField))
                {
                    // Load all parameters for a sonar
                    nh->getParam(sonarField, id);
                    nh->getParam(topicField, topic);
                    nh->getParam(thresholdField, threshold);
                    nh->getParam(historyLengthField, historyLength);

                    // Add sonar to sonars map
                    Sonar sonar(threshold, historyLength, id, topic);
                    std::pair<std::string, Sonar> sonarPair(id, sonar);
                    sonars.insert(sonarPair);

                    ++i;
                }
                else
                {
                    // Error in configuration - no threshold for this sonar
                    ROS_ERROR_STREAM_NAMED("sonar_obstacle_detection_node",
                        topicField << ", " << thresholdField << " or "
                        << historyLengthField << " unspecified for "
                        << sonarField << ". Exiting...");

                    ros::shutdown();
                }
            }
            else
            {
                allParametersLoaded = true;
            }
        }

        ROS_INFO_STREAM_NAMED("sonar_obstacle_detection_node",
                             "Loaded " << i << " sonars.");
    }

    void run()
    {
        ros::spin();
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_obstacle_detection_node");
    ros::NodeHandle nh("~");
    SonarObstacleDetectionNode sodn(&nh);
    sodn.run();
}
