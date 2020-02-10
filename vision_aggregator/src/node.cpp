#include <vision_aggregator/vision_aggregator.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_aggregator_node");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    vision_aggregator::VisionAggregator a(nh, pnh);
    ros::waitForShutdown();
    return 0;
}
