#include <hazmat_detection/hazmat_detection.h>

HazmatDetection::HazmatDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it_(nh), nh_(nh), pnh_(pnh)
{
    enable_srv_ = nh_.advertiseService("/mercury/softwares/hazmat", &HazmatDetection::setEnableSrvCallback, this);
    perception_pub_ = nh_.advertise<amrl_vision_common::Perceptions>("/vision/perceptions", 5);
    image_sub_ = it_.subscribe("image", 1, &HazmatDetection::imageCallback, this);
}

HazmatDetection::~HazmatDetection()
{
}

bool HazmatDetection::setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res)
{
    if (req.enabled)
    {
        if (!is_enabled)
        {
            image_sub_ = it_.subscribe("image", 1, &HazmatDetection::imageCallback, this);
            is_enabled = true;
        }
    }
    else
    {
        if (is_enabled)
        {
            image_sub_.shutdown();
            is_enabled = false;
        }
    }
    res.state = is_enabled;
}

void HazmatDetection::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cam_image;
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    nn.detect(cam_image->image);
}