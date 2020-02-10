#include <vision_aggregator/vision_aggregator.h>

#include <boost/thread/lock_guard.hpp>

namespace vision_aggregator
{
VisionAggregator::VisionAggregator(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh), storage_duration_(ros::Duration(1))
{
    image_transport::SubscriberStatusCallback connect_cb = boost::bind(&VisionAggregator::connectCb, this);
    image_sub_ = it_.subscribe("image", 1, &VisionAggregator::imageCallback, this);
    perception_sub_ = nh.subscribe("/vision/perceptions", 1, &VisionAggregator::perceptionCallback, this);

    cv::namedWindow("view", cv::WINDOW_NORMAL);
    cv::startWindowThread();
}

VisionAggregator::~VisionAggregator() {}

void VisionAggregator::imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image;
    int image_height_ = image.rows;
    int image_width_ = image.cols;

    removeUnusedPerceptions();

    for (auto &percept_pair : detection_map)
    {
        amrl_vision_common::Perceptions &percept_list = percept_pair.second;
        std::string name = percept_list.perception_name;
        cv::Scalar color = cv::Scalar(255, 255, 255);
        if (name == "motion") color = cv::Scalar(255, 0, 0);
        else if (name == "qrcode") color = cv::Scalar(0, 255, 0);
        else if (name == "hazmat") color = cv::Scalar(0, 0, 255);
        for (amrl_vision_common::Perception &percept : percept_list.perceptions)
        {
            std::vector<cv::Point> polygon;

            cv::Point first_point;
            first_point.x = image_width_;
            first_point.y = image_height_;
            for (geometry_msgs::Point32 &ros_point : percept.polygon.points)
            {
                cv::Point cv_point;
                cv_point.x = ros_point.x * image_width_;
                cv_point.y = ros_point.y * image_height_;
                polygon.push_back(cv_point);
                if (cv_point.x < first_point.x && cv_point.y < first_point.y)
                    first_point = cv_point;
            }
            const cv::Point *pts = (const cv::Point *)cv::Mat(polygon).data;
            int npts = cv::Mat(polygon).rows;

            cv::polylines(image, &pts, &npts, 1, true, color, 1, CV_AA, 0);
            first_point.y -= 5;
            if (percept.name != "")
                name = percept.name;
            cv::putText(image, name, first_point, CV_FONT_HERSHEY_SIMPLEX, 0.75, color, 1);
        }
    }
    cv::imshow("view", image);
    cv::waitKey(30);
}

void VisionAggregator::perceptionCallback(const amrl_vision_common::PerceptionsConstPtr &msg)
{
    detection_map[msg->perception_name] = *msg;
}

void VisionAggregator::removeUnusedPerceptions()
{
    ros::Time storage_threshold;
    if ((ros::Time::now().toSec() > storage_duration_.toSec()))
        storage_threshold = ros::Time::now() - storage_duration_;

    std::vector<std::string> keys;
    for (std::map<std::string, amrl_vision_common::Perceptions>::iterator it = detection_map.begin(); it != detection_map.end(); ++it)
        keys.push_back(it->first);

    for (std::string &key : keys)
    {
        const amrl_vision_common::Perceptions &detection_array = detection_map[key];
        if (detection_array.header.stamp.toSec() < storage_threshold.toSec())
        {
            auto it = detection_map.find(key);
            detection_map.erase(it);
        }
    }
}

void VisionAggregator::connectCb()
{
    ROS_WARN("connect callback");
}

} // namespace vision_aggregator