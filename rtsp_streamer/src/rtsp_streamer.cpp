#include <rtsp_streamer/rtsp_streamer.h>

RTSPStreamer::RTSPStreamer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh)
{
    int port;
    pnh_.getParam("port", port);
    if (port == 0)
        port = 8080;

    writer = MJPEGWriter(port);
    writer.start();
}

RTSPStreamer::~RTSPStreamer()
{
    writer.stop();
}

void RTSPStreamer::spin()
{
    std::string rtsp_link;
    pnh_.getParam("rtsp_link", rtsp_link);
    cv::VideoCapture capture(rtsp_link);
    cv::Mat frame;
    while (ros::ok())
    {
        capture >> frame;
        if (frame.empty())
            break;

        writer.write(frame);
    }
}
