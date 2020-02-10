#include <qrcode_detection/qrcode_reader.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>
#include <tf/tfMessage.h>
#include <amrl_qrcode_detection/QRCode.h>
#include <amrl_qrcode_detection/QRCodes.h>

class ObjectTfPub
{
private:
    ros::Publisher pub_tf_;
    ros::Publisher pub_tf_private_;
    ros::Publisher pub_items_private_;

public:
    ObjectTfPub(ros::NodeHandle nh, ros::NodeHandle private_nh);
    void publish(std::vector<barcode::Barcode> &barcodes, const sensor_msgs::ImageConstPtr &msg) const;
};