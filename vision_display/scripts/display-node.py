#!/usr/bin/env python

import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from amrl_qrcode_detection.msg import QRCodes
from amrl_vision_common.msg import Boxes
from amrl_vision_common.srv import SetEnabled


class Display:
    def __init__(self):
        rospy.Subscriber('image', Image, self.image_callback, queue_size=1)
        self.motion_subscriber = rospy.Subscriber(
            '/vision/motion_detection/boxes',
            Boxes,
            self.motion_callback
        )
        self.qrcode_subscriber = rospy.Subscriber(
            '/vision/qrcode_detection/qrcodes',
            QRCodes,
            self.qrcode_callback
        )
        self.hazmat_subscriber = rospy.Subscriber(
            '/vision/hazmat_detections/hazmats',
            Boxes,
            self.hazmat_callback
        )
        self.publisher = rospy.Publisher(
            '/mercury/display/image', Image, queue_size=1)
        self.bridge = CvBridge()

        self.motion_boxes = []
        self.motion_lasttime = 0
        self.qrcode_boxes = []
        self.qrcode_lasttime = 0
        self.hazmat_boxes = []
        self.hazmat_lasttime = 0
    
    def hazmat_callback(self, data):
        self.hazmat_boxes = data.boxes
        self.hazmat_lasttime = time.time()

    def qrcode_callback(self, data):
        self.qrcode_boxes = data.qrcodes
        self.qrcode_lasttime = time.time()

    def motion_callback(self, data):
        self.motion_boxes = data.boxes
        self.motion_lasttime = time.time()

    def image_callback(self, data):
        if self.publisher.get_num_connections() == 0:
            return

        # image = cv2.imdecode(np.fromstring(data.data, np.uint8), cv2.IMREAD_COLOR)
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.resize(image, (0, 0), fx=0.5, fy=0.5)
        height, width = image.shape[:2]        

        for box in self.motion_boxes:
            x, y, w, h = box.x, box.y, box.w, box.h
            x = int(x * width)
            w = int(w * width)
            y = int(y * height)
            h = int(h * height)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 1)

        for box in self.hazmat_boxes:
            x, y, w, h = box.x, box.y, box.w, box.h
            x = int(x * width)
            w = int(w * width)
            y = int(y * height)
            h = int(h * height)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv2.putText(image, box.name, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        for box in self.qrcode_boxes:
            corners = box.corners
            for i in range(4):
                corners[i].x = int(corners[i].x * width)
                corners[i].y = int(corners[i].y * height)
            cv2.line(
                image,
                (corners[0].x, corners[0].y),
                (corners[1].x, corners[1].y),
                (255, 0, 0), 1
            )
            cv2.line(
                image,
                (corners[1].x, corners[1].y),
                (corners[2].x, corners[2].y),
                (255, 0, 0), 1
            )
            cv2.line(
                image,
                (corners[2].x, corners[2].y),
                (corners[3].x, corners[3].y),
                (255, 0, 0), 1
            )
            cv2.line(
                image,
                (corners[3].x, corners[3].y),
                (corners[0].x, corners[0].y),
                (255, 0, 0), 1
            )            

        current_time = time.time()
        if self.motion_boxes:
            if current_time - self.motion_lasttime > 1:
                self.motion_boxes = []
        if self.qrcode_boxes:
            if current_time - self.qrcode_lasttime > 1:
                self.qrcode_boxes = []
        if self.hazmat_boxes:
            if current_time - self.hazmat_lasttime > 1:
                self.hazmat_boxes = []
                
        self.publisher.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))


if __name__ == "__main__":
    rospy.init_node("amrl_vision_display", anonymous=False)
    d = Display()
    rospy.spin()
