#include <qrcode_detection/object_tf.h>

ObjectTfPub::ObjectTfPub(ros::NodeHandle nh, ros::NodeHandle private_nh) : pub_tf_(nh.advertise<tf::tfMessage>("/tf", 5)),
                                                                           pub_tf_private_(private_nh.advertise<tf::tfMessage>("tf_objects", 5)),
                                                                           pub_items_private_(nh.advertise<amrl_qrcode_detection::QRCodes>("/vision/qrcode_detection/qrcodes", 5))
{
}

void ObjectTfPub::publish(std::vector<barcode::Barcode> &barcodes, const sensor_msgs::ImageConstPtr &msg) const
{
    tf::tfMessage tf_msg;
    geometry_msgs::TransformStamped tr;
    tr.header = msg->header;

    amrl_qrcode_detection::QRCodes qrcodes;
    for (uint i = 0; i < barcodes.size(); i++)
    {
        amrl_qrcode_detection::QRCode qrcode;

        barcode::Barcode &barcode = barcodes[i];
        tr.child_frame_id = barcode.data;
        tr.transform.rotation.w = 1;
        tr.transform.translation.x = barcode.x;
        tr.transform.translation.y = barcode.y;
        tr.transform.translation.z = barcode.z;
        tf_msg.transforms.push_back(tr);

        qrcode.name = barcode.data;
        for (int i = 0; i < 4; i++)
        {
            amrl_vision_common::Point point;
            point.x = barcode.corners[i].x;
            point.y = barcode.corners[i].y;
            qrcode.corners.push_back(point);
        }
        qrcodes.qrcodes.push_back(qrcode);
    }
    if (pub_tf_.getNumSubscribers())
        pub_tf_.publish(tf_msg);
    if (pub_tf_private_.getNumSubscribers())
        pub_tf_private_.publish(tf_msg);
    if (pub_items_private_.getNumSubscribers())
        pub_items_private_.publish(qrcodes);
}