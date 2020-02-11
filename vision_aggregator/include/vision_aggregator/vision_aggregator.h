#ifndef VISION_AGGREGATOR_H_
#define VISION_AGGREGATOR_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <amrl_vision_common/Perceptions.h>

namespace vision_aggregator
{
class VisionAggregator
{
public:
    VisionAggregator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~VisionAggregator();
    std::string getTime();
private:
    void imageCallback(const sensor_msgs::ImageConstPtr &img);
    void perceptionCallback(const amrl_vision_common::PerceptionsConstPtr &msg);
    void removeUnusedPerceptions();
    std::string last_time;

    ros::Duration storage_duration_;
    std::map<std::string, amrl_vision_common::Perceptions> detection_map;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    ros::Subscriber perception_sub_;
    image_transport::Subscriber image_sub_;    
    image_transport::Publisher image_pub_;
};
} // namespace vision_aggregator
#endif