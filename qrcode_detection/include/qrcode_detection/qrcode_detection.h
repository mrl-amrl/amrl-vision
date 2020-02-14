#include <qrcode_detection/qrcode_reader.h>
#include <qrcode_detection/visualizer.h>
#include <qrcode_detection/object_tf.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <amrl_qrcode_detection/QRCodeConfig.h>
#include <amrl_vision_common/SetEnabled.h>
#include <std_msgs/String.h>

class Node
{
private:
    barcode::BarcodeReader reader_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;

    Visualizer vis_;
    ObjectTfPub object_pub_;
    ros::ServiceServer enable_srv_;
    ros::Publisher string_pub_;
    bool enabled_;

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void callback(amrl_qrcode_detection::QRCodeConfig &config, uint32_t level);
    bool setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res);

public:
    Node(ros::NodeHandle nh, ros::NodeHandle nh_private);
};
