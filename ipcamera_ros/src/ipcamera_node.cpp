#include "ipcamera/ipcamera_node.hpp"
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>

IPCamera::IPCamera(ros::NodeHandle *nodeHandle) : nh_(nodeHandle),
                                                        imagetransport_(*nodeHandle)
{
    nh_->param<std::string>("video_url", video_url_, "http://192.168.10.33:80/mjpeg?video.mjpg");
    ROS_INFO_STREAM("video_url  " << video_url_);
    nh_->param<std::string>("frame_id", frame_id_, "camera");
    camera_pub_ = imagetransport_.advertiseCamera("/ip_camera/" + frame_id_, 10);
    refresh_serviceServer_ = nh_->advertiseService("refresh", &IPCamera::refreshSrvCallback, this);
    cap_.open(video_url_);
}

IPCamera::~IPCamera()
{
}

bool IPCamera::publish()
{
    cv::Mat frame;
    ros::Rate loop(30);
    while (ros::ok())
    {
        if (cap_.isOpened())
        {
            ROS_INFO_ONCE("[amrl_ipcamera] connection established");
            if (camera_pub_.getNumSubscribers() > 0)
            {
                cap_ >> frame;
                if (frame.empty())
                    continue;
                cv_bridge::CvImage out_msg;
                out_msg.header.frame_id = frame_id_;
                out_msg.header.stamp = ros::Time::now();
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = frame;
                sensor_msgs::CameraInfo camera_info;
                sensor_msgs::Image rosimg;
                out_msg.toImageMsg(rosimg);
                camera_pub_.publish(rosimg, camera_info, ros::Time::now());
            }
        }
        else
        {
            cap_.release();
            ROS_ERROR_STREAM("[amrl_ipcamera] IP Camera is not avaliable");
            cap_.open(video_url_);
        }

        ros::spinOnce();
        loop.sleep();
    }
    return true;
}

bool IPCamera::refreshSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("[amrl_ipcamera] refreshing");
    cap_.release();
    if (!cap_.open(video_url_))
    {
        ROS_ERROR_STREAM("[amrl_ipcamera] Connecting to " << video_url_ << " failed.");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ipcamera_node");
    ros::NodeHandle nh("~");
    IPCamera ipCamera(&nh);
    ipCamera.publish();
    return 0;
}
