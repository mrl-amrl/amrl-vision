#ifndef HAZMAT_DETECTION_H
#define HAZMAT_DETECTION_H

#include <string>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <tf/tfMessage.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <hazmat_detection/neural_network.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point32.h>
#include <amrl_vision_common/SetEnabled.h>
#include <amrl_vision_common/Perception.h>
#include <amrl_vision_common/Perceptions.h>

class HazmatDetection
{
public:
    HazmatDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~HazmatDetection();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr &img);
    void pointsCallback(const sensor_msgs::PointCloud2Ptr &points);
    bool setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res);
    
    int counter_;
    int skip_frames;
    std::string frame_id;

    NeuralNetwork nn;
    bool is_enabled;
    bool pcl_enabled = false;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::ServiceServer enable_srv_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    ros::Subscriber pcl_sub_;
    ros::Publisher tf_pub_;

    ros::Publisher perception_pub_;
};

#endif