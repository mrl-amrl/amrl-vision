#include <hazmat_detection/hazmat_detection.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hazmat_detection_node");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    HazmatDetection a(nh, pnh);
    ros::waitForShutdown();
    return 0;
}
