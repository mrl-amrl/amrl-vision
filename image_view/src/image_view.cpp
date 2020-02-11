#include <image_view/image_view.h>

ImageView::ImageView(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh)
{
    image_sub_ = it_.subscribe("image", 1, &ImageView::imageCallback, this);
    created_ = false;
    pnh_.getParam("window_name", window_name);
    if (window_name == "")
        window_name = ros::this_node::getName();
}

ImageView::~ImageView()
{
    cv::destroyWindow(window_name);
}

void ImageView::imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image;
    if (!created_)
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

    cv::imshow(window_name, image);
    cv::waitKey(1);
}
