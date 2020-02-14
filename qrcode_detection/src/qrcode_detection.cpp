

#include <qrcode_detection/qrcode_detection.h>

void Node::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_img_ptr;
    msg->header;

    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(msg, "mono8");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR_STREAM("Could not convert ROS image to CV: " << e.what());
        return;
    }

    static int id = 0;
    int n = reader_.parse(cv_img_ptr);
    ROS_DEBUG_STREAM("Got image, has " << n << " barcodes");
    std::vector<barcode::Barcode> barcodes = reader_.getBarcodes();
    for (uint i = 0; i < reader_.getBarcodes().size(); i++)
    {
        ROS_DEBUG_STREAM("Barcode: " << barcodes[i].data
                                     << " x:" << barcodes[i].x
                                     << " y:" << barcodes[i].y);
        vis_.publish(barcodes[i].x, barcodes[i].y, barcodes[i].z, msg->header.frame_id, id++ % 1000);
    }
    if (msg->header.frame_id == "")
    {
        ROS_ERROR_THROTTLE(1, "Received image with empty frame_id, would cause tf connectivity issues.");
    }
    if (barcodes.size() == 1 && string_pub_.getNumSubscribers() > 0)
    {
        std_msgs::String data;
        data.data = barcodes.at(0).data;
        string_pub_.publish(data);
    }
    object_pub_.publish(barcodes, msg);
}

void Node::callback(amrl_qrcode_detection::QRCodeConfig &config, uint32_t level)
{
    reader_.setBarcodeSize(config.barcode_size).setFOV(config.fov);
}

bool Node::setEnableSrvCallback(amrl_vision_common::SetEnabled::Request &req, amrl_vision_common::SetEnabled::Response &res)
{
    enabled_ = req.enabled;
    if (req.enabled)
    {

        ROS_INFO_STREAM("Setting QR detection enabled");
        sub_image_ = it_.subscribe("image", 1, &Node::imageCallback, this);
    }
    else
    {
        ROS_INFO_STREAM("Setting QR detection disabled");
        sub_image_.shutdown();
    }
    res.state = enabled_;
}

Node::Node(ros::NodeHandle nh, ros::NodeHandle nh_private) : it_(nh),
                                                             object_pub_(nh, nh_private)
{
    dynamic_reconfigure::Server<amrl_qrcode_detection::QRCodeConfig> server;
    dynamic_reconfigure::Server<amrl_qrcode_detection::QRCodeConfig>::CallbackType f;
    f = boost::bind(&Node::callback, this, _1, _2);
    server.setCallback(f);

    enable_srv_ = nh.advertiseService("/mercury/softwares/qrcode", &Node::setEnableSrvCallback, this);
    string_pub_ = nh.advertise<std_msgs::String>("/mercury/gui/qr_code", 10);
}
