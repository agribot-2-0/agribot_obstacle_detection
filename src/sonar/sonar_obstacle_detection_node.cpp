#include <ros/ros.h>
#include <sensor_msgs/Range.h>

class SonarObstacleDetectionNode
{
 private:
    // ROS handlers
    ros::NodeHandle *nh;

    // Sonar map with topic and threshold
    std::map<std::string, float> sonars;

    // Sonar map with topic and threshold
    std::map<std::string, std::shared_ptr<ros::Subscriber>> subscribers;

 public:
    explicit SonarObstacleDetectionNode(ros::NodeHandle *_nh) :
        nh(_nh)
    {
        // Load sonar parameters
        loadSonars();

        // Register sonar subscribers
        registerSubscribers();
    }

    ~SonarObstacleDetectionNode()
    {
    }

    void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,
                       std::string topic)
    {
        float threshold = sonars[topic];
        float range = msg->range;

        if (range < threshold)
        {
            // Obstacle detected
            obstacleDetected(topic, range);
        }
    }

    void obstacleDetected(std::string topic, float range)
    {
        ROS_ERROR_STREAM("Obstacle detected in sonar: " << topic
                         << " at range: " << range);
    }

    void registerSubscribers()
    {
        for (auto it = sonars.begin(); it != sonars.end(); ++it)
        {
            std::string topic = it->first;

            ROS_INFO_STREAM_NAMED("sonar_obstacle_detection_node",
                                  "Subscribing to topic: " << topic);

            // Subscribe
            std::shared_ptr<ros::Subscriber> sub;
            sub = std::make_shared<ros::Subscriber>();
            (*sub) = nh->subscribe<sensor_msgs::Range>(
                topic,
                1,
                boost::bind(&SonarObstacleDetectionNode::sonarCallback,
                            this, _1, topic));

            // Add subscriber to subscribers map
            std::pair<std::string, std::shared_ptr<ros::Subscriber>> pair(
                topic, sub);
            subscribers.insert(pair);
        }
    }

    void run()
    {
        ros::spin();
    }

    void loadSonars()
    {
        bool allParametersLoaded = false;
        std::size_t i = 0;
        std::string sonarField, thresholdField;
        std::string topic;
        float threshold;

        while (!allParametersLoaded)
        {
            sonarField = "sonar" + std::to_string(i);
            thresholdField = sonarField + "_threshold";

            if (nh->hasParam(sonarField))
            {
                if (nh->hasParam(thresholdField))
                {
                    // Load both parameters
                    nh->getParam(sonarField, topic);
                    nh->getParam(thresholdField, threshold);
                    ++i;

                    // Add sonar to map
                    std::pair<std::string, float> sonar(topic, threshold);
                    sonars.insert(sonar);
                }
                else
                {
                    // Error in configuration - no threshold for this sonar
                    ROS_ERROR_STREAM_NAMED("sonar_obstacle_detection_node",
                                           "No threshold for " << sonarField
                                           << ". Exiting...");
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
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_obstacle_detection_node");
    ros::NodeHandle nh("~");
    SonarObstacleDetectionNode sodn(&nh);
    sodn.run();
}
