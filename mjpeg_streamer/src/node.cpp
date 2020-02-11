#include <mjpeg_streamer/mjpeg_streamer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_view_node");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    MJPEGStreamer a(nh, pnh);
    ros::waitForShutdown();
    return 0;
}
