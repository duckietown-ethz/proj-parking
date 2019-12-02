#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import (BoolStamped, FSMState, Segment,
    SegmentList, Vector2D)
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils.jpg import bgr_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker

from line_detector.timekeeper import TimeKeeper
import cv2
import rospy
import threading
import time
from line_detector.line_detector_plot import color_segment, drawLines
import numpy as np


class LineDetectorNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"

        # Constructor of line detector
        self.bridge = CvBridge()

        self.active = True

        self.stats = Stats()

        # Only be verbose every 10 cycles
        self.intermittent_interval = 10000
        self.intermittent_counter = 0

        # these will be added if it becomes verbose
        self.pub_edge = None
        self.pub_colorSegment = None

        # We use two different detectors (parameters) for lane following and
        # intersection navigation (both located in yaml file)
        self.detector = None
        self.detector_intersection = None
        self.detector_used = self.detector


        self.verbose = None
        self.updateParams(None)

        self.fsm_state = "NORMAL_JOYSTICK_CONTROL"

        # Publishers
        self.pub_lines = rospy.Publisher("~segment_list", SegmentList, queue_size=1)
        self.pub_image = rospy.Publisher("~image_with_lines", Image, queue_size=1)

        # Subscribers
        self.sub_image = rospy.Subscriber("~corrected_image/compressed", CompressedImage, self.processImage, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm = rospy.Subscriber("~fsm_mode", FSMState, self.cbFSM, queue_size=1)

        rospy.loginfo("[%s] Initialized (verbose = %s)." %(self.node_name, self.verbose))

        rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)


    def cbFSM(self, msg):
        self.fsm_state = msg.state

        if self.fsm_state in ["INTERSECTION_PLANNING", "INTERSECTION_COORDINATION", "INTERSECTION_CONTROL"]:
            self.detector_used = self.detector_intersection
        else:
            self.detector_used = self.detector


    def updateParams(self, _event):
        old_verbose = self.verbose
        self.verbose = rospy.get_param('~verbose', True)
        # self.loginfo('verbose = %r' % self.verbose)
        if self.verbose != old_verbose:
            self.loginfo('Verbose is now %r' % self.verbose)

        self.image_size = rospy.get_param('~img_size')
        self.top_cutoff = rospy.get_param('~top_cutoff')
        self.top_cutoff = 40

        if self.detector is None:
            c = rospy.get_param('~detector')
            assert isinstance(c, list) and len(c) == 2, c

#         if str(self.detector_config) != str(c):
            self.loginfo('new detector config: %s' % str(c))

            self.detector = instantiate(c[0], c[1])
#             self.detector_config = c
            self.detector_used = self.detector

        if self.detector_intersection is None:
            c = rospy.get_param('~detector_intersection')
            assert isinstance(c, list) and len(c) == 2, c

#         if str(self.detector_config) != str(c):
            self.loginfo('new detector_intersection config: %s' % str(c))

            self.detector_intersection = instantiate(c[0], c[1])
#             self.detector_config = c

        if self.verbose and self.pub_edge is None:
            self.pub_edge = rospy.Publisher("~edge", Image, queue_size=1)
            self.pub_colorSegment = rospy.Publisher("~colorSegment", Image, queue_size=1)


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def intermittent_log_now(self):
        return self.intermittent_counter % self.intermittent_interval == 1

    def intermittent_log(self, s):
        if not self.intermittent_log_now():
            return
        self.loginfo('%3d:%s' % (self.intermittent_counter, s))

    def processImage(self, image_msg):
        self.stats.received()

        if not self.active:
            return

        try:
            self.processImage_(image_msg)
        finally:
            return

    def processImage_(self, image_msg):

        self.stats.processed()

        if self.intermittent_log_now():
            self.intermittent_log(self.stats.info())
            self.stats.reset()

        tk = TimeKeeper(image_msg)

        self.intermittent_counter += 1

        # Decode from compressed image with OpenCV
        try:
            image_cv = bgr_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        tk.completed('decoded')

        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]

        if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            image_cv = cv2.resize(image_cv, (self.image_size[1], self.image_size[0]),
                                   interpolation=cv2.INTER_NEAREST)
        image_cv = image_cv[self.top_cutoff:,:,:]

        tk.completed('resized')

        # Set the image to be detected
        self.detector_used.setImage(image_cv)

        # Detect lines and normals

        white = self.detector_used.detectLines('white')
        yellow = self.detector_used.detectLines('yellow')
        red = self.detector_used.detectLines('red')
        #added for parking viciopoli01
        green = self.detector_used.detectLines('green')
        blue = self.detector_used.detectLines('blue')

        tk.completed('detected')

        # SegmentList constructor
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp

        # Convert to normalized pixel coordinates, and add segments to segmentList
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.image_size[1], 1./self.image_size[0], 1./self.image_size[1], 1./self.image_size[0]))
        if len(white.lines) > 0:
            lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_white, white.normals, Segment.WHITE))
            #print("THIS IS WHAT THEY MEAN FOR COLOR : "+str(Segment.WHITE))
        if len(yellow.lines) > 0:
            lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_yellow, yellow.normals, Segment.YELLOW))
        if len(red.lines) > 0:
            lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_red, red.normals, Segment.RED))
        #added for parking viciopoli01
        if len(green.lines) > 0:
            lines_normalized_green = ((green.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_green, green.normals, 3))
        if len(blue.lines) > 0:
            lines_normalized_blue = ((blue.lines + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized_blue, blue.normals, 4))

        self.intermittent_log('# segments: white %3d yellow %3d red %3d green %3d blue %3d' % (len(white.lines),
                len(yellow.lines), len(red.lines), len(green.lines), len(blue.lines)))

        tk.completed('prepared')

        # Publish segmentList
        self.pub_lines.publish(segmentList)
        tk.completed('--pub_lines--')

        # VISUALIZATION only below

        if self.verbose:

            # print('line_detect_node: verbose is on!')

            # Draw lines and normals
            image_with_lines = np.copy(image_cv)
            drawLines(image_with_lines, white.lines, (0, 0, 0))
            drawLines(image_with_lines, yellow.lines, (255, 0, 0))
            drawLines(image_with_lines, red.lines, (0, 255, 0))
            drawLines(image_with_lines, green.lines, (0,255,0))
            drawLines(image_with_lines, blue.lines, (0,0,255))

            tk.completed('drawn')

            # Publish the frame with lines
            image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
            image_msg_out.header.stamp = image_msg.header.stamp
            self.pub_image.publish(image_msg_out)

            tk.completed('pub_image')

#         if self.verbose:

            colorSegment = color_segment(white.area, red.area, yellow.area, green.area, blue.area)
            edge_msg_out = self.bridge.cv2_to_imgmsg(self.detector_used.edges, "mono8")
            colorSegment_msg_out = self.bridge.cv2_to_imgmsg(colorSegment, "bgr8")
            self.pub_edge.publish(edge_msg_out)
            self.pub_colorSegment.publish(colorSegment_msg_out)

            tk.completed('pub_edge/pub_segment')


        self.intermittent_log(tk.getall())


    def onShutdown(self):
        self.loginfo("Shutdown.")

    def toSegmentMsg(self,  lines, normals, color):

        segmentMsgList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y

            segmentMsgList.append(segment)
        return segmentMsgList

class Stats():
    def __init__(self):
        self.nresets = 0
        self.reset()

    def reset(self):
        self.nresets += 1
        self.t0 = time.time()
        self.nreceived = 0
        self.nskipped = 0
        self.nprocessed = 0

    def received(self):
        if self.nreceived == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node received first image.')
        self.nreceived += 1

    def skipped(self):
        self.nskipped += 1

    def processed(self):
        if self.nprocessed == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node processing first image.')

        self.nprocessed += 1

    def info(self):
        delta = time.time() - self.t0

        if self.nreceived:
            skipped_perc = (100.0 * self.nskipped / self.nreceived)
        else:
            skipped_perc = 0

        def fps(x):
            return '%.1f fps' % (x / delta)

        m = ('In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' %
             (delta, self.nreceived, fps(self.nreceived),
              self.nprocessed, fps(self.nprocessed),
              self.nskipped, fps(self.nskipped), skipped_perc))
        return m





if __name__ == '__main__':
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
rospy.spin()
