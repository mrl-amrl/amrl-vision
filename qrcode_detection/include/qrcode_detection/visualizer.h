#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

class Visualizer
{
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    visualization_msgs::Marker marker_;

public:
    Visualizer();
    void publish(const geometry_msgs::PoseStamped &target, int id);
    void publish(float x, float y, float z, const std::string &frame_id, int id);
};