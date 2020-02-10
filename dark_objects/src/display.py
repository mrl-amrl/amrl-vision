#!/usr/bin/env python
import cv2
import rospy
import message_filters
import numpy as np
from nms import non_max_suppression_fast
from imutils import resize
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from amrl_vision_common.srv import SetEnabled
from amrl_vision_common.msg import Box, Boxes


class Display:
    def __init__(self):
        self.bridge = CvBridge()

        box_subscriber = message_filters.Subscriber('/image/boxes', Boxes)
        image_subscriber = message_filters.Subscriber(
            '/usb_cam/image_raw',
            Image
        )
        ts = message_filters.TimeSynchronizer(
            [image_subscriber, box_subscriber],
            100
        )
        ts.registerCallback(self._callback)

    def _callback(self, image, boxes):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        height, width = image.shape[:2]

        boxes = boxes.boxes
        for box in boxes:
            x = int(box.x * width)
            y = int(box.y * height)
            w = int(box.w * width)
            h = int(box.h * height)
            cv2.rectangle(image, (x, y), (x+w, y+h), (200, 100, 100), 2)

        cv2.imshow('image', image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.shutdown()

    def shutdown(self):
        cv2.destroyAllWindows()
        rospy.logwarn('shutting down...')
        rospy.signal_shutdown('close')


if __name__ == "__main__":
    rospy.init_node('dark_object_display')
    display = Display()
    rospy.on_shutdown(display.shutdown)
    rospy.spin()
