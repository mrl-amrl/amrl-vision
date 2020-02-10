#include <vision_aggregator/vision_aggregator.h>

#include <boost/thread/lock_guard.hpp>

namespace vision_aggregator
{
VisionAggregator::VisionAggregator(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh)
{
    image_transport::SubscriberStatusCallback connect_cb = boost::bind(&VisionAggregator::connectCb, this);
    image_sub_ = it_.subscribe("image", 1, &VisionAggregator::imageCallback, this);
    perception_sub_ = nh.subscribe("/vision/perceptions", 1, &VisionAggregator::perceptionCallback, this);

    boost::lock_guard<boost::mutex> lock(connect_mutex_);

    cv::namedWindow("view");
    cv::startWindowThread();
}

VisionAggregator::~VisionAggregator() {}

void VisionAggregator::imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv::imshow("view", cv_bridge::toCvShare(img, "bgr8")->image);
    cv::waitKey(30);
}

void VisionAggregator::perceptionCallback(const amrl_vision_common::PerceptionsConstPtr &msg)
{
    if (msg->perception_name == "motion")
    {
        ROS_WARN("motion");
    }
}

void VisionAggregator::connectCb()
{
    ROS_WARN("connect callback");
}

} // namespace vision_aggregator