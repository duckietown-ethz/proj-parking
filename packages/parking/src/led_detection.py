#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import rospkg
import rospy

from duckietown_utils import load_camera_intrinsics
from duckietown_msgs.msg import BoolStamped
from sensor_msgs.msg import CompressedImage, Image

WIDTH = 320
HEIGHT = 240


class LEDDetectionNode(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh = os.environ['VEHICLE_NAME']
        self.veh_name = rospy.get_namespace().strip("/")
        self.bridge = CvBridge()
        self.active = True
        self.config = self.setupParam("~config", "baseline")
        self.publish_freq = self.setupParam("~publish_freq", 2.0)
        self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.last_stamp = rospy.Time.now()
        self.frontorback = "back" #CHANGE TO CHECK BOTH!!#os.environ.get("FRONT_OR_BACK")
        rospack = rospkg.RosPack()

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
        self.Midtold = 0
        self.depthold = 0
        self.Midtoldb = 0
        self.deptholdb = 0
        self.time = rospy.get_rostime().to_sec()
        self.timeb = rospy.get_rostime().to_sec()


    def bottomOfImage(self, full_image):
        if self.look_right:
            return full_image[HEIGHT//2-40:HEIGHT//2+30, WIDTH//2:WIDTH-10]
        else:
            return full_image[HEIGHT//2-40:HEIGHT//2+15,10:WIDTH//2]


    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data


    def cbLookRight(self, msg):
        look_right = msg.data
        rospy.loginfo('[%s] look_right = %s' % (self.node_name, look_right))
        self.look_right = look_right


    def undistort(self, keypoints):
        #undistort radially
        #section initUndistortRectifyMap from https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html
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
            # print(e)
            return []


    def processImage(self, image_msg):
        if not self.active:
            return

        img = self.bottomOfImage(self.readImage(image_msg))
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        cv_image_color = img

        now = rospy.Time.now()

        cv_image_color = cv_image_color[cv_image_color.shape[0]/4:cv_image_color.shape[0]/4*3]

        cv_image1 = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)
        #cv_image1 = cv_image1[cv_image1.shape[0]/4:cv_image1.shape[0]/4*3]

        ret, cv_image = cv2.threshold(cv_image1, 220, 255, cv2.THRESH_BINARY)

        # Set up the detector with default parameters.
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 10 # the graylevel of images
        params.maxThreshold = 200

        params.filterByColor = True
        params.blobColor = 255

        # Filter by Area
        params.filterByArea = True
        params.minArea = 10
        params.filterByInertia = False
        params.filterByConvexity = False
        params.filterByCircularity = True
        params.minCircularity = 0.9
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
        redfound = 0
        whitefound = 0

        for i,key1 in enumerate(keypoints_un):
            pixel1 = cv_image_color[int(keypoints[i].pt[1]), int(keypoints[i].pt[0])]

            blue1 = pixel1[0]
            bluethreshold = 240
            # Check if the blue value of the led light is matching the red back or the white front
            if (blue1 < bluethreshold): # red
                redfound = 1
            if  (blue1 >= bluethreshold): #white
                whitefound = 1

        if redfound == 1:
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.red_pub.publish(msg)

        else:
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = False
            self.red_pub.publish(msg)

        if self.publish_circles and whitefound == 0:
            # cv2.drawChessboardCorners(image_cv,
            #                             self.circlepattern_dims, corners, detection)
                    # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
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
