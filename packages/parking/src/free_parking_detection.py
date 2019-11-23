#!/usr/bin/env python
import rospy
import cv2
import os

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList, Vector2D
from duckietown_msgs.msg import BoolStamped

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

from duckietown import DTROS


class FreeParking(DTROS):
    """
        This node publishs a boolean value
        if the parking spot is free the value is true, false otherwise
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(FreeParking, self).__init__(node_name=node_name)
        self.veh = os.environ['VEHICLE_NAME']

        # Adjust camera resolution
        rospy.set_param('/'+self.veh+'/camera_node/res_w', 320)
        rospy.set_param('/'+self.veh+'/camera_node/res_h', 240)
        rospy.set_param('/'+self.veh+'/camera_node/exposure_mode', 'off')

        self.updateParameters()

        # defining the topic names
        free_topic = "~/"+self.veh+"/free_parking/"
        out_image_topic = "~/"+self.veh+"/camera_node/free_parking_image/compressed"
        camera_topic="/"+self.veh+"/camera_node/image/compressed"

        # init the publisher
        self.is_free = rospy.Publisher(free_topic, BoolStamped, queue_size=1)
        #self.out_image = rospy.Publisher(out_image_topic, CompressedImage, queue_size=1)
        #Setup the camera subscriber
        self.camera = rospy.Subscriber(camera_topic, CompressedImage, self.detectGreen)

        self.bridge = CvBridge()

        self.hsv_green1=np.array([45, 100, 100])
        self.hsv_green2=np.array([75, 255, 255])
        self.dilation_kernel_size=3
        self.edges = np.empty(0)


    def detectGreen(self,data):
        img = self.readImage(data)
        img = img[160:,:120]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #detect green
        bw = cv2.inRange(hsv, self.hsv_green1, self.hsv_green2)

        # binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)

        msg = BoolStamped()
        msg.header.stamp=rospy.Time.now()

        if np.sum(bw/255)>300:
            msg.data=True
            self.is_free.publish(msg)
        else :
            msg.data=False
            self.is_free.publish(msg)


    def readImage(self,msg_image):
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
    camera_node = FreeParking(node_name='parking_free')
    # Keep it spinning to keep the node alive
    rospy.spin()
