#!/usr/bin/env python

"""
This node looks in the bottom of the image for
the color white and publishes a Boolean value
if it detects an amount of white above a certain threshold.
----------
It is used by parking as a trigger that the Duckiebot
can stop the parking maneuver when it approaches a white line.
"""

import rospy
import cv2
import os
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown import DTROS

WIDTH = 320 # Width of the image
HEIGHT = 240 # Height of the image

# Cropping bounds for cropping images and detecting white lines
CROP = (HEIGHT-10, HEIGHT, 0, WIDTH//2+50)


class WhiteLineDetectorNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(WhiteLineDetectorNode, self).__init__(node_name=node_name)
        self.veh = os.environ['VEHICLE_NAME']

        # Adjust camera resolution
        rospy.set_param('/%s/camera_node/res_w' % self.veh, WIDTH)
        rospy.set_param('/%s/camera_node/res_h' % self.veh, HEIGHT)
        rospy.set_param('/%s/camera_node/exposure_mode' % self.veh, 'off')

        self.updateParameters()

        # Publishers
        self.white_pub = rospy.Publisher(
            '~/%s/parking/white_line' % self.veh,
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
        self.hsv_white1 = np.array([0, 0, 150])
        self.hsv_white2 = np.array([180, 100, 255])
        self.dilation_kernel_size = 3


    def croppedImage(self, full_image):
        return full_image[CROP[0]:CROP[1], CROP[2]:CROP[3]]


    def detectColor(self, data):
        img = self.croppedImage(self.readImage(data))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Detect white
        bw = cv2.inRange(hsv, self.hsv_white1, self.hsv_white2)

        # Binary dilation
        kernel_size = (self.dilation_kernel_size, self.dilation_kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        bw = cv2.dilate(bw, kernel)
        color_count = np.sum(bw / 255)
        detected = (color_count > self.detection_threshold)

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = detected
        self.white_pub.publish(msg)


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
            # print(e)
            return []


if __name__ == '__main__':
    # Initialize the node
    detector = WhiteLineDetectorNode(node_name='white_line_detector')
    # Keep it spinning to keep the node alive
    rospy.spin()
