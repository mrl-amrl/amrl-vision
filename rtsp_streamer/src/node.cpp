#include <rtsp_streamer/rtsp_streamer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtsp_streamer_node");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    RTSPStreamer a(nh, pnh);
    a.spin();
    return 0;
}
