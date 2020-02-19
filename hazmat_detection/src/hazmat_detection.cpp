#include <hazmat_detection/hazmat_detection.h>

HazmatDetection::HazmatDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it_(nh), nh_(nh), pnh_(pnh)
{
    std::string cfg_path, weights_path, labels_path;
    pnh_.getParam("cfg_path", cfg_path);
    pnh_.getParam("weights_path", weights_path);
    pnh_.getParam("labels_path", labels_path);
    pnh_.getParam("skip_frames", skip_frames);
    pnh_.getParam("camera_frame_id", frame_id);
    nn = NeuralNetwork(cfg_path, weights_path, labels_path);

    enable_srv_ = nh_.advertiseService("/mercury/softwares/hazmat", &HazmatDetection::setEnableSrvCallback, this);
    perception_pub_ = nh_.advertise<amrl_vision_common::Perceptions>("/vision/perceptions", 5);
    counter_ = 0;
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
    if (perception_pub_.getNumSubscribers() == 0)
        return;
    if (counter_++ < skip_frames)
        return;
    counter_ = 0;

    cv_bridge::CvImagePtr cam_image;
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    int image_height_ = cam_image->image.rows;
    int image_width_ = cam_image->image.cols;

    std::vector<detection_rect> rects = nn.detect(cam_image->image);
    if (rects.size() == 0)
        return;

    amrl_vision_common::Perceptions perceptions;
    perceptions.header.stamp = ros::Time::now();
    perceptions.header.frame_id = frame_id;
    perceptions.perception_name = "hazmat";
    amrl_vision_common::Perception perception;

    for (int i = 0; i < rects.size(); i++)
    {
        detection_rect item = rects[i];

        float x = (float)item.rect.x / image_width_;
        float y = (float)item.rect.y / image_height_;
        float w = (float)item.rect.width / image_width_;
        float h = (float)item.rect.height / image_height_;
        geometry_msgs::Point32 p1, p2, p3, p4;
        p1.x = x;
        p1.y = y;
        p2.x = x + w;
        p2.y = y;
        p3.x = x + w;
        p3.y = y + h;
        p4.x = x;
        p4.y = y + h;
        std::vector<geometry_msgs::Point32> points;
        points.push_back(p1);
        points.push_back(p2);
        points.push_back(p3);
        points.push_back(p4);

        perception.name = item.name;
        perception.polygon.points = points;
        perceptions.perceptions.push_back(perception);
    }
    perception_pub_.publish(perceptions);
}