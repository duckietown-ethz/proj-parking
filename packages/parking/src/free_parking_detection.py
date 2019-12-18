#!/usr/bin/env python

"""
This node is used by parking to detect a free parking spot.
It works by checking the lower-left corner of the image
for green pixels, and if enough green pixels are seen, it
publishes a message `True` (otherwise publishes `False`).
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

# Require to see more green when exiting parking spot
EXITING_THRESHOLD = 400
# Less green needed when searching for parking spot
SEARCHING_THRESHOLD = 115

# Cropping bounds for cropping images and performing parking spot detection
EXITING_CROP    =    (130,  HEIGHT-30,  0,  WIDTH)
SEARCHING_CROP  =    (160,  HEIGHT,     0,  120)


class FreeParking(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(FreeParking, self).__init__(node_name=node_name)
        self.veh = os.environ['VEHICLE_NAME']

        # Adjust camera resolution
        rospy.set_param('/%s/camera_node/res_w' % self.veh, WIDTH)
        rospy.set_param('/%s/camera_node/res_h' % self.veh, HEIGHT)
        rospy.set_param('/%s/camera_node/exposure_mode' % self.veh, 'off')

        self.is_reversing = False

        self.updateParameters()

        # Publishers
        self.is_free_pub = rospy.Publisher(
            '~/%s/parking/free_parking' % self.veh,
            BoolStamped,
            queue_size=1
        )

        # Subscribers
        self.camera = rospy.Subscriber(
            '/%s/camera_node/image/compressed' % self.veh,
            CompressedImage,
            self.detectColor
        )
        self.reverse_sub = rospy.Subscriber(
            '/%s/parking/reverse' % self.veh,
            BoolStamped,
            self.cbReverse,
            queue_size=1
        )

        self.bridge = CvBridge()
        self.detection_threshold = 115
        self.detect_green = True
        self.hsv_green1 = np.array([45, 100, 100])
        self.hsv_green2 = np.array([75, 255, 255])
        self.hsv_blue1 = np.array([90, 100, 100])
        self.hsv_blue2 = np.array([150, 255, 255])
        self.dilation_kernel_size = 3
        self.edges = np.empty(0)


    def cbReverse(self, msg):        
        is_reversing = msg.data
        tup = (self.node_name, is_reversing)
        rospy.loginfo('[%s] Reverse = %s' % tup)
        self.is_reversing = is_reversing
        self.detection_threshold = EXITING_THRESHOLD if is_reversing else SEARCHING_THRESHOLD


    def croppedImage(self, full_image):
        crop = EXITING_CROP if self.is_reversing else SEARCHING_CROP
        return full_image[crop[0]:crop[1], crop[2]:crop[3]]


    def detectColor(self, data):
        img = self.croppedImage(self.readImage(data))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Detect the color
        lower_bound = self.hsv_green1 if self.detect_green else self.hsv_blue1
        upper_bound = self.hsv_green2 if self.detect_green else self.hsv_blue2
        bw = cv2.inRange(hsv, lower_bound, upper_bound)

        # Binary dilation
        kernel_size = (self.dilation_kernel_size, self.dilation_kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        bw = cv2.dilate(bw, kernel)
        color_count = np.sum(bw / 255)
        detected = (color_count > self.detection_threshold)

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = detected
        self.is_free_pub.publish(msg)


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
            print(e)
            return []


if __name__ == '__main__':
    # Initialize the node
    free_parking_node = FreeParking(node_name='parking_free')
    # Keep it spinning to keep the node alive
    rospy.spin()
