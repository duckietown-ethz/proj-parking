#!/usr/bin/env python
import rospy
import cv2
import os
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import BoolStamped
from duckietown import DTROS

WIDTH = 320
HEIGHT = 240

class PinkLedDetectorNode(DTROS):
    """
        This node looks in the bottom of the image for
        the color pink and publishes a Boolean value
        if it detects an amount of pink above a certain threshold.
        ----------
        It is used by parking as a trigger that the Duckiebot
        can stop the parking maneuver when it approaches a pink line.
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(PinkLedDetectorNode, self).__init__(node_name=node_name)
        self.veh = os.environ['VEHICLE_NAME']

        # Adjust camera resolution
        rospy.set_param('/%s/camera_node/res_w' % self.veh, WIDTH)
        rospy.set_param('/%s/camera_node/res_h' % self.veh, HEIGHT)
        rospy.set_param('/%s/camera_node/exposure_mode' % self.veh, 'off')

        self.updateParameters()

        # Publishers
        self.pink_pub = rospy.Publisher(
            '~/%s/parking/pink_line' % self.veh,
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
        self.hsv_pink1 = np.array([150, 150, 100])
        self.hsv_pink2 = np.array([160, 255, 255])
        self.dilation_kernel_size = 3
        self.edges = np.empty(0)


    def croppedImage(self, full_image):
        return full_image[HEIGHT-3:, :WIDTH//2]


    def detectColor(self, data):
        img = self.croppedImage(self.readImage(data))
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Detect pink
        bw = cv2.inRange(hsv, self.hsv_pink1, self.hsv_pink2)

        # Binary dilation
        kernel_size = (self.dilation_kernel_size, self.dilation_kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        bw = cv2.dilate(bw, kernel)
        color_count = np.sum(bw / 255)
        detected = (color_count > self.detection_threshold)

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = detected
        self.pink_pub.publish(msg)


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
    detector = PinkLedDetectorNode(node_name='pink_led_detector')
    # Keep it spinning to keep the node alive
    rospy.spin()
