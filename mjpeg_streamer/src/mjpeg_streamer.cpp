#include <mjpeg_streamer/mjpeg_streamer.h>

MJPEGStreamer::MJPEGStreamer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), it_(nh)
{
    int port;
    pnh_.getParam("port", port);
    if (port == 0) port = 8080;
    
    image_sub_ = it_.subscribe("image", 1, &MJPEGStreamer::imageCallback, this);
    writer = MJPEGWriter(port);
    writer.start();
}

MJPEGStreamer::~MJPEGStreamer()
{
    writer.stop();
}

void MJPEGStreamer::imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv::Mat image = cv_bridge::toCvShare(img, "bgr8")->image;
    int image_height_ = image.rows;
    int image_width_ = image.cols;
    if (image_width_ > 480)
    {
        cv::resize(image, image, cv::Size(480, 320));
        image_width_ = 480;
        image_height_ = 320;
    }
    writer.write(image);
}
