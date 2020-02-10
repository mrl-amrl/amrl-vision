#include <iostream>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <motion_detection/motion_detection.h>

using namespace cv;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_detection_node");

    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    MotionDetection a(&nh);
    ros::waitForShutdown();
    return 0;
}
