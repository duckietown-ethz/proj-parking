#!/usr/bin/env python

"""
This node is used by parking to detect LEDs of other
Duckiebots. It works by using blob detection from OpenCV.
The results are published as True if LEDs are detected
and False otherwise; a debug image is also published
and is viewable on rqt.
"""

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import rospkg
import rospy

from duckietown_utils import load_camera_intrinsics
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64

WIDTH = 320 # Width of the image
HEIGHT = 240 # Height of the image

# White threshold for cv2 binary thresholding
WHITE_THRESHOLD = 235
# Minimum area of blobs for the blob detector
MIN_AREA = 1
# Minimum circularity of blobs (1 = perfect circle)
MIN_CIRCULARITY = 0.3

# Cropping bounds for cropping images and performing blob detection
RIGHT_CROP  =    (HEIGHT//2-40,  HEIGHT//2+30,   WIDTH//2,   WIDTH)
LEFT_CROP   =    (HEIGHT//2-40,  HEIGHT//2+15,   0,          WIDTH//2)


class LEDDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh = os.environ['VEHICLE_NAME']
        self.veh_name = rospy.get_namespace().strip("/")
        self.bridge = CvBridge()
        self.active = True
        self.publish_circles = True
        self.look_right = False

        # Subscribers
        self.sub_image = rospy.Subscriber(
            '~/%s/camera_node/image/compressed' % self.veh,
            CompressedImage,
            self.processImage,
            buff_size=921600,
            queue_size=1
        )
        self.sub_look_right = rospy.Subscriber(
            '~/%s/parking/led_detection_right' % self.veh_name,
            BoolStamped,
            self.cbLookRight,
            queue_size=1
        )

        # Publishers
        self.pub_circlepattern_image = rospy.Publisher(
            "~circlepattern_image",
            Image,
            queue_size=1
        )
        self.red_pub = rospy.Publisher(
            '~/%s/parking/red_led' % self.veh,
            BoolStamped,
            queue_size=1
        )

        self.intrinsics = load_camera_intrinsics(self.veh_name)
        self.fx = self.intrinsics['K'][0][0]
        self.fy = self.intrinsics['K'][1][1]
        self.radialparam = self.intrinsics['D']
        self.time = rospy.get_rostime().to_sec()


    def bottomOfImage(self, full_image):
        crop = RIGHT_CROP if self.look_right else LEFT_CROP
        return full_image[crop[0]:crop[1], crop[2]:crop[3]]


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data


    def cbLookRight(self, msg):
        look_right = msg.data
        rospy.loginfo('[%s] look_right = %s' % (self.node_name, look_right))
        self.look_right = look_right


    def undistort(self, keypoints):
        # undistort radially
        # section initUndistortRectifyMap from https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
        for key in keypoints:
            k1 = self.radialparam[0][0]
            k2 = self.radialparam[0][1]
            p1 = self.radialparam[0][2]
            p2 = self.radialparam[0][3]
            imheight, imwidth = self.cv_image.shape[:2]
            cx = imwidth/2
            cy = imheight/2
            xr = (key.pt[0]-cx)/self.fx
            yr = (key.pt[1]-cy)/self.fy
            rd = np.sqrt(xr*xr+yr*yr)
            xun = cx+self.fx*(xr*(1+k1*rd*rd+k2*rd*rd*rd*rd)+2*p1*xr*yr+p2*(rd*rd+2*xr*xr))
            yun = cy+self.fy*(yr*(1+k1*rd*rd+k2*rd*rd*rd*rd)+p1*(rd*rd+2*yr*yr)+2*p2*xr*yr)
            key.pt = (xun,yun)
            key.size = key.size

        return keypoints


    def features_deepcopy(self,f):
        # because deepcopy does not work with keypoint
        return [cv2.KeyPoint(x = k.pt[0], y = k.pt[1],
                _size = k.size, _angle = k.angle,
                _response = k.response, _octave = k.octave,
                _class_id = k.class_id) for k in f]


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


    def processImage(self, image_msg):
        if not self.active:
            return

        img = self.bottomOfImage(self.readImage(image_msg))

        cv_image_color = img

        now = rospy.Time.now()

        cv_image_color = cv_image_color[cv_image_color.shape[0]/4:cv_image_color.shape[0]/4*3]
        cv_image_hsv = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2HSV)
        cv_image1 = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)
        ret, cv_image = cv2.threshold(cv_image1, WHITE_THRESHOLD, 255, cv2.THRESH_BINARY)

        # Set up the detector with default parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10 # the graylevel of images
        params.maxThreshold = 200

        params.filterByColor = True
        params.blobColor = 255

        # Filter by Area
        params.filterByArea = True
        params.minArea = MIN_AREA
        params.filterByInertia = False
        params.filterByConvexity = False
        params.filterByCircularity = True
        params.minCircularity = MIN_CIRCULARITY
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(cv_image)
        # Undistort radially the points
        self.cv_image = cv_image
        keyin = self.features_deepcopy(keypoints)
        keypoints_un = self.undistort(keyin)

        x1 = x2 = y1 = y2 = 0
        x1d = x2d = y1d = y2d = 0
        x1b = x2b = y1b = y2b = 0
        x1db = x2db = y1db = y2db = 0
        redfound = len(keypoints_un) > 0

        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = redfound
        self.red_pub.publish(msg)

        if self.publish_circles:
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            cv_image1 = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            #front
            cv_image1 = cv2.circle(cv_image1,(int(x1),int(y1)), 5, (0,255,0), 2)
            cv_image1 = cv2.circle(cv_image1,(int(x2),int(y2)), 5, (0,255,0), 2)
            cv_image1 = cv2.circle(cv_image1,(int(x1d),int(y1d)), 5, (255,0,0), 2)
            cv_image1 = cv2.circle(cv_image1,(int(x2d),int(y2d)), 5, (255,0,0), 2)
            #back
            cv_image1 = cv2.circle(cv_image1,(int(x1b),int(y1b)), 5, (0,100,0), 2)
            cv_image1 = cv2.circle(cv_image1,(int(x2b),int(y2b)), 5, (0,100,0), 2)
            cv_image1 = cv2.circle(cv_image1,(int(x1db),int(y1db)), 5, (100,0,0), 2)
            cv_image1 = cv2.circle(cv_image1,(int(x2db),int(y2db)), 5, (100,0,0), 2)

            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image1, "bgr8")
            self.pub_circlepattern_image.publish(image_msg_out)


if __name__ == '__main__':
    rospy.init_node('led_detection', anonymous=False)
    led_detection_node = LEDDetectionNode()
    rospy.spin()
