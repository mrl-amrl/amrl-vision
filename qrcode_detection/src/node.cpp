#include <ros/ros.h>
#include <qrcode_detection/qrcode_detection.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrcode_detection");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    Node node(ros::NodeHandle(), ros::NodeHandle("~"));
    ros::waitForShutdown();
    return 0;
}
