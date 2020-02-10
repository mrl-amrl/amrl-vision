#include "qrcode_detection/visualizer.h"

Visualizer::Visualizer() : marker_pub_(nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1))
{
    marker_.ns = "qrcodes";
    marker_.type = visualization_msgs::Marker::CUBE;
}

void Visualizer::publish(float x, float y, float z, const std::string &frame_id, int id)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    pose.pose.orientation.w = 1;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    publish(pose, id);
}

void Visualizer::publish(const geometry_msgs::PoseStamped &target, int id)
{
    if (marker_pub_.getNumSubscribers() == 0)
        return;
    marker_.id = id;
    marker_.header.stamp = target.header.stamp;
    marker_.header.frame_id = target.header.frame_id;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose = target.pose;

    marker_.scale.x = 0.05;
    marker_.scale.y = 0.05;
    marker_.scale.z = 0.05;
    marker_.color.r = 0.7;
    marker_.color.g = 0.7;
    marker_.color.b = 0.7;
    marker_.color.a = 0.1;
    marker_.lifetime = ros::Duration(0);
    marker_pub_.publish(marker_);
}