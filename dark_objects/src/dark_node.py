#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import os
from nms import non_max_suppression_fast
from inspect import getframeinfo, stack
from imutils import resize
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from amrl_vision_common.srv import SetEnabled
from amrl_vision_common.msg import Box, Boxes
from amrl_dark_objects.cfg import DarkObjectsConfig


class DarkObjectsDetector:
    def __init__(self):
        self.pub = rospy.Publisher(
            '/image/boxes',
            Boxes,
            queue_size=int(rospy.get_param("~queue_size", '10'))
        )
        self.bridge = CvBridge()
        # TODO: make it reconfigureable
        self.kernel = np.ones((5, 5), np.uint8)
        self.min_area = 12
        self.min_height = 96
        self.min_width = 128
        self.min_solidity = 0.5
        self._image_sub = None

        DynamicReconfigureServer(DarkObjectsConfig, self.configuration)
        rospy.Service(
            '/{}/set_enable'.format(rospy.get_name()),
            SetEnabled,
            self.set_enable_cb
        )

    def set_enable(self, state):
        if state:
            self._image_sub = rospy.Subscriber(
                '/usb_cam/image_raw', Image, self._callback)
        elif self._image_sub is not None:
            self._image_sub.unregister()

    def set_enable_cb(self, req):
        self.set_enable(req.enabled)
        return req.enabled

    def configuration(self, config, level):
        self.min_area = config['min_area']
        self.min_solidity = config['min_solidity']
        self.crop_offset = config['crop_offset']
        return config

    def get_crop_values(self, width, height):
        ix = int((float(self.crop_offset) / width) * 100)
        iy = int((float(self.crop_offset) / height) * 100)
        return ix, iy

    def crop_img(self, image):
        height, width = image.shape[:2]
        ix, iy = self.get_crop_values(width, height)
        image = image[
            iy:height - iy,
            ix:width - ix
        ]
        return image

    def filter_boxes(self, contours, width, height):
        ix, iy = self.get_crop_values(width, height)
        width += ix * 2
        height += iy * 2

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            solidity = area / (w * h)
            if solidity < self.min_solidity:
                continue
            if w > 128 and h > 96:
                continue
            boxes.append([x, y, x+w, y+h])

        boxes = non_max_suppression_fast(np.array(boxes), 0.3).tolist()
        for i, box in enumerate(boxes):
            box[0] += ix
            box[1] += iy
            box[2] += ix
            box[3] += iy

            box[0] = float(box[0]) / width
            box[1] = float(box[1]) / height
            box[2] = float(box[2]) / width
            box[3] = float(box[3]) / height
            boxes[i] = box
        return boxes

    def box_to_msg(self, box):
        b = Box()
        b.x = box[0]
        b.y = box[1]
        b.w = box[2] - b.x
        b.h = box[3] - b.y
        return b

    def _callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = self.crop_img(image)
        height, width = image.shape[:2]

        _, threshold = cv2.threshold(image, 80, 255, cv2.THRESH_BINARY)
        threshold = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(
            threshold,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        boxes = self.filter_boxes(contours, width, height)
        boxes = map(self.box_to_msg, boxes)

        msg = Boxes()
        msg.header = data.header
        msg.boxes = boxes
        self.pub.publish(msg)

    def shutdown(self):
        rospy.logwarn('shutting down...')


if __name__ == "__main__":
    rospy.init_node('dark_object_detector')
    dark_obj = DarkObjectsDetector()
    rospy.on_shutdown(dark_obj.shutdown)
    rospy.spin()
