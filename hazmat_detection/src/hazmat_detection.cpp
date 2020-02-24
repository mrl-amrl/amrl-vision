#include <hazmat_detection/hazmat_detection.h>

HazmatDetection::HazmatDetection(ros::NodeHandle &nh, ros::NodeHandle &pnh) : it_(nh), nh_(nh), pnh_(pnh)
{
    std::string cfg_path, weights_path, labels_path;
    pnh_.getParam("cfg_path", cfg_path);
    pnh_.getParam("weights_path", weights_path);
    pnh_.getParam("labels_path", labels_path);
    pnh_.getParam("skip_frames", skip_frames);
    pnh_.getParam("camera_frame_id", frame_id);
    if (!pnh_.getParam("pcl_enabled", pcl_enabled))
        pcl_enabled = false;

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
            if (!pcl_enabled)
                image_sub_ = it_.subscribe("image", 1, &HazmatDetection::imageCallback, this);
            else
            {
                pcl_sub_ = nh_.subscribe("points", 10, &HazmatDetection::pointsCallback, this);
                tf_pub_ = nh_.advertise<tf::tfMessage>("tf", 10);
            }
            is_enabled = true;
        }
    }
    else
    {
        if (is_enabled)
        {
            if (!pcl_enabled)
                image_sub_.shutdown();
            else
            {
                pcl_sub_.shutdown();
                tf_pub_.shutdown();
            }
            is_enabled = false;
        }
    }
    res.state = is_enabled;
}

void HazmatDetection::pointsCallback(const sensor_msgs::PointCloud2Ptr &msg)
{
    if (perception_pub_.getNumSubscribers() == 0 && tf_pub_.getNumSubscribers() == 0)
        return;

    sensor_msgs::Image image;
    pcl::toROSMsg(*msg, image);

    cv_bridge::CvImagePtr cam_image;
    cam_image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    int image_height_ = cam_image->image.rows;
    int image_width_ = cam_image->image.cols;
    std::vector<detection_rect> rects = nn.detect(cam_image->image);
    if (rects.size() == 0)
        return;

    // one or more hazmat has been detected, lets try to find object in 3D model.
    pcl::PointCloud<pcl::PointXYZ> depth;
    pcl::fromROSMsg(*msg, depth);

    amrl_vision_common::Perceptions perceptions;
    perceptions.header.stamp = ros::Time::now();
    perceptions.header.frame_id = frame_id;
    perceptions.perception_name = "hazmat";
    amrl_vision_common::Perception perception;

    tf::tfMessage tf_msg;
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

        int center_x = item.rect.x + item.rect.width / 2;
        int center_y = item.rect.y + item.rect.height / 2;
        pcl::PointXYZ point_from_cloud = depth.at(center_x, center_y);

        geometry_msgs::TransformStamped tr;
        tr.header = msg->header;
        tr.child_frame_id = "hazmat_" + item.name + "_" + std::to_string(i);
        tr.transform.rotation.w = 1;
        if (!std::isnan(point_from_cloud.z))
        {
            tr.transform.translation.x = point_from_cloud.x;
            tr.transform.translation.y = point_from_cloud.y;
            tr.transform.translation.z = point_from_cloud.z;
        }
        else
        {
            tr.transform.translation.x = 0;
            tr.transform.translation.y = 0;
            tr.transform.translation.z = 0.2;
        }
        tf_msg.transforms.push_back(tr);

        perception.name = item.name;
        perception.polygon.points = points;
        perceptions.perceptions.push_back(perception);
    }

    if (tf_msg.transforms.size() != 0)
        tf_pub_.publish(tf_msg);
    perception_pub_.publish(perceptions);
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