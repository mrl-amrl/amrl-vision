#include <image_view/image_view.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_view_node");

    ros::NodeHandle nh("");
    ros::NodeHandle pnh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ImageView a(nh, pnh);
    ros::waitForShutdown();
    return 0;
}
