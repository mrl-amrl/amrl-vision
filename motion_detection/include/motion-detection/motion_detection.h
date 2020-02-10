#ifndef MOTION_DETECTION_H_
#define MOTION_DETECTION_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <amrl_vision_common/SetEnabled.h>
#include <amrl_vision_common/Perception.h>
#include <amrl_vision_common/Perceptions.h>
#include <amrl_motion_detection/MotionDetectionConfig.h>


class MotionDetection
{

public:
    MotionDetection(ros::NodeHandle *nodehandle);
    bool setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res);
    void callbackDynamicCfg(amrl_motion_detection::MotionDetectionConfig &config, uint32_t level);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

private:
    bool is_enable_;
    bool _is_enabled = false;
    ros::NodeHandle *nh_;
    ros::ServiceServer enable_srv_;
    ros::Publisher perception_pub_;
    dynamic_reconfigure::Server<amrl_motion_detection::MotionDetectionConfig>::CallbackType f_;
    dynamic_reconfigure::Server<amrl_motion_detection::MotionDetectionConfig> server_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Ptr<cv::BackgroundSubtractorMOG2> bg_subtractor_;
    float learning_rate_;
    int erosion_iterations_,dilation_iterations_;
    int morphology_kernel_size_,dilation_kernel_size_;
    int min_area_;
    amrl_vision_common::Perceptions perceptions_;
    amrl_vision_common::Perception perception_;
    int image_width_,image_height_;
};

#endif
