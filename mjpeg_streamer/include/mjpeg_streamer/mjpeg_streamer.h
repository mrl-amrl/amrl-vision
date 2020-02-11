#ifndef VISION_AGGREGATOR_H_
#define VISION_AGGREGATOR_H_

#include <ros/ros.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <mjpeg_streamer/mjpeg_writer.h>

class MJPEGStreamer
{
public:
    MJPEGStreamer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~MJPEGStreamer();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr &img);
    MJPEGWriter writer;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};
#endif