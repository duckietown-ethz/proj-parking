#!/usr/bin/env python

"""
This node detects the color red at the bottom of an image,
and publishes a boolean `True` if enough red is seen, otherwise
it publishes False. It is used by parking to determine if the
Duckiebot is at an intersection within the parking area.
"""

import rospy
import cv2
import os
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import BoolStamped

from duckietown import DTROS

WIDTH = 320 # Width of the image
HEIGHT = 240 # Height of the image
# Cropping bounds for cropping images and detecting red lines
CROP = (HEIGHT-5, HEIGHT, WIDTH//2-50, WIDTH//2+50)


class RedLine(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(RedLine, self).__init__(node_name=node_name)
        self.veh = os.environ['VEHICLE_NAME']

        # Adjust camera resolution
        rospy.set_param('/%s/camera_node/res_w' % self.veh, WIDTH)
        rospy.set_param('/%s/camera_node/res_h' % self.veh, HEIGHT)
        rospy.set_param('/%s/camera_node/exposure_mode' % self.veh, 'off')

        self.updateParameters()

        # Publishers
        self.red_pub = rospy.Publisher(
            "~/%s/red_line" % self.veh,
            BoolStamped,
            queue_size=1
        )

        # Subscribers
        self.camera = rospy.Subscriber(
            '/%s/camera_node/image/compressed' % self.veh,
            CompressedImage,
            self.detectColor
        )

        self.bridge = CvBridge()
        self.detection_threshold = 300
        self.hsv_red1 = np.array([0, 140, 100])
        self.hsv_red2 = np.array([15, 255, 255])
        self.hsv_red3 = np.array([165, 140, 100])
        self.hsv_red4 = np.array([180, 255, 255])
        self.dilation_kernel_size = 3


    def croppedImage(self, full_image):
        return full_image[CROP[0]:CROP[1], CROP[2]:CROP[3]]


    def detectColor(self, data):
        img = self.croppedImage(self.readImage(data))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # detect red
        bw1 = cv2.inRange(hsv, self.hsv_red1, self.hsv_red2)
        bw2 = cv2.inRange(hsv, self.hsv_red3, self.hsv_red4)
        bw = cv2.bitwise_or(bw1, bw2)

        # binary dilation
        kernel_size = (self.dilation_kernel_size, self.dilation_kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        bw = cv2.dilate(bw, kernel)
        color_count = np.sum(bw / 255)
        detected = (color_count > self.detection_threshold)

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = detected
        self.red_pub.publish(msg)


    def readImage(self, msg_image):
        """Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            return []


if __name__ == '__main__':
    # Initialize the node
    detector_node = RedLine(node_name='red_line')
    # Keep it spinning to keep the node alive
    rospy.spin()
