#!/usr/bin/env python
import rospy
import cv2
import os

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Segment, SegmentList, Vector2D
from duckietown_msgs.msg import BoolStamped
from std_msgs.msg import Int16

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

from duckietown import DTROS

class FreeExit(DTROS):
    """
        This node publishs a boolean value
        if the parking spot is free the value is true, false otherwise
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(FreeExit, self).__init__(node_name=node_name)
        self.veh = os.environ['VEHICLE_NAME']

        # Adjust camera resolution
        rospy.set_param('/'+self.veh+'/camera_node/res_w', 320)
        rospy.set_param('/'+self.veh+'/camera_node/res_h', 240)
        rospy.set_param('/'+self.veh+'/camera_node/exposure_mode', 'off')

        self.updateParameters()
        # defining the topic names
        free_topic = "~/"+self.veh+"/free_exit/"
        out_image_topic = "~/"+self.veh+"/camera_node/free_spot_image/compressed"
        camera_topic="/"+self.veh+"/camera_node/image/compressed"

        # Publishers
        self.is_free_pub = rospy.Publisher(free_topic, BoolStamped, queue_size=1)


        self.pub_img_out = rospy.Publisher(out_image_topic, CompressedImage, queue_size=1)
        # Subscribers
        self.camera = rospy.Subscriber(camera_topic, CompressedImage, self.detectColor)


        red1="/"+self.veh+"/red1"
        red2="/"+self.veh+"/red2"
        red3="/"+self.veh+"/red3"
        red4="/"+self.veh+"/red4"
        rospy.Subscriber(red1, Int16, self.red1)
        rospy.Subscriber(red2, Int16, self.red2)
        rospy.Subscriber(red3, Int16, self.red3)
        rospy.Subscriber(red4, Int16, self.red4)

        self.bridge = CvBridge()
        self.bridge = CvBridge()
        self.detection_threshold = 250
        self.hsv_red1 = np.array([0,140,100])
        self.hsv_red2 = np.array([15,255,255])
        self.hsv_red3 = np.array([165,140,100])
        self.hsv_red4 = np.array([180,255,255])
        self.dilation_kernel_size = 3
        self.edges = np.empty(0)

    def red1(self,msg):
        self.hsv_red1 = np.array([msg,140,100])

    def red2(self,msg):
        self.hsv_red1 = np.array([msg,255,255])

    def red3(self,msg):
        self.hsv_red1 = np.array([msg,140,100])

    def red4(self,msg):
        self.hsv_red1 = np.array([msg,255,255])


    def detectColor(self,data):
        img = self.readImage(data)

        img = img[:150,60:160] # for forward exiting :150,:160

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #detect red
        bw1 = cv2.inRange(hsv, self.hsv_red1, self.hsv_red2)
        bw2 = cv2.inRange(hsv, self.hsv_red3, self.hsv_red4)
        bw = cv2.bitwise_or(bw1, bw2)


        # binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)

        self.pub_img_out.publish(self.bridge.cv2_to_compressed_imgmsg(bw))

        color_count = np.sum(bw/255)
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
    camera_node = FreeExit(node_name='free_exit')
    # Keep it spinning to keep the node alive
    rospy.spin()
