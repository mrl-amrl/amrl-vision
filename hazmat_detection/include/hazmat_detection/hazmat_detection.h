#ifndef HAZMAT_DETECTION_H
#define HAZMAT_DETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <hazmat_detection/neural_network.h>
#include <image_transport/image_transport.h>
#include <amrl_vision_common/SetEnabled.h>
#include <amrl_vision_common/Perceptions.h>

class HazmatDetection
{
public:
    HazmatDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~HazmatDetection();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr &img);
    bool setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res);

    NeuralNetwork nn;
    bool is_enabled;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer enable_srv_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher perception_pub_;
};

#endif