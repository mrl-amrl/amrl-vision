#include <image_view/image_view.h>

ImageView::ImageView(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh)
{
    image_sub_ = it_.subscribe("image", 1, &ImageView::imageCallback, this);
    created_ = false;
}

ImageView::~ImageView()
{
    cv::destroyWindow("AMRL Image View");
}

void ImageView::imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image;
    if (!created_)
        cv::namedWindow("AMRL Image View", cv::WINDOW_NORMAL);

    cv::imshow("AMRL Image View", image);
    cv::waitKey(1);
}
