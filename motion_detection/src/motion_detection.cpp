#include "motion_detection/motion_detection.h"

MotionDetection::MotionDetection(ros::NodeHandle *nodehandle) : nh_(nodehandle),
                                                                it_(*nodehandle)

{
    bg_subtractor_ = cv::createBackgroundSubtractorMOG2();
    enable_srv_ = nh_->advertiseService("/mercury/softwares/motion", &MotionDetection::setEnableSrvCallback, this);
    f_ = boost::bind(&MotionDetection::callbackDynamicCfg, this, _1, _2);
    server_.setCallback(f_);
    perceptions_.perception_name = "motion";
    perceptions_.header.frame_id = "motion";
    perception_pub_ = nh_->advertise<amrl_vision_common::Perceptions>("/vision/perceptions", 5);
}

void MotionDetection::callbackDynamicCfg(amrl_motion_detection::MotionDetectionConfig &config, uint32_t level)
{
    learning_rate_ = config.learning_rate;
    bg_subtractor_->setHistory(config.history);
    bg_subtractor_->setDetectShadows(config.shadow);
    erosion_iterations_ = config.erosion_iterations;
    dilation_iterations_ = config.dilation_iterations;
    min_area_ = config.min_area;
    morphology_kernel_size_ = config.morphology_kernel_size;
    dilation_kernel_size_ = config.dilation_kernel_size;
}

bool MotionDetection::setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res)
{
    if (req.enabled)
    {
        if (!_is_enabled)
        {
            image_sub_ = it_.subscribe("image", 1, &MotionDetection::imageCallback, this);
            _is_enabled = true;
        }
    }
    else
    {
        if (_is_enabled)
        {
            image_sub_.shutdown();
            _is_enabled = false;
        }
    }
    res.state = _is_enabled;
}

void MotionDetection::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if (perception_pub_.getNumSubscribers() == 0)
        return;

    cv_bridge::CvImageConstPtr cv_img_ptr;
    cv_img_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat frame;

    frame = cv_img_ptr->image;

    image_height_ = frame.rows;
    image_width_ = frame.cols;

    cv::Mat fgimg;

    bg_subtractor_->apply(frame, fgimg, learning_rate_);

    cv::Mat fgimg_orig;

    fgimg.copyTo(fgimg_orig);
    cv::morphologyEx(fgimg, fgimg, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morphology_kernel_size_, morphology_kernel_size_)));
    for (int i = 0; i < erosion_iterations_; i++)
        cv::erode(fgimg, fgimg, cv::Mat());

    cv::Mat dialate_elements;
    dialate_elements = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilation_kernel_size_, dilation_kernel_size_));
    for (int i = 0; i < dilation_iterations_; i++)
        cv::dilate(fgimg, fgimg, dialate_elements);

    cv::Rect bounding_rect;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    if (contours.size() == 0)
        return;

    int area;
    perceptions_.perceptions.clear();
    perceptions_.header.stamp = ros::Time::now();
    perception_.polygon.points.clear();
    for (int i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point> hull;
        area = cv::contourArea(contours[i]);
        if (area > min_area_)
        {
            std::vector<geometry_msgs::Point32> points;
            convexHull( contours[i], hull,true);
            for (int i = 0; i < hull.size(); i ++)
            {
               cv::Point p;
               p = hull.at(i);
               float x = (float)p.x / image_width_ ;
               float y = (float)p.y / image_height_;
               geometry_msgs::Point32 point;
               point.x = x;
               point.y = y;
               points.push_back(point);
            }
            perception_.polygon.points = points;
            perceptions_.perceptions.push_back(perception_);
        }

    perception_pub_.publish(perceptions_);
    }
}
