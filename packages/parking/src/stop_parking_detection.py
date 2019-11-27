#!/usr/bin/env python
import rospy
import numpy as np
from duckietown import DTROS
from duckietown_msgs.msg import (
    SegmentList,
    Segment,
    BoolStamped,
    LanePose
)
from geometry_msgs.msg import Point
import math

"""
This is a simplified version of stop_line_filter_node.py
from the package `stop_line_filter` in dt-core
"""
class StopParkingDetectionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(StopParkingDetectionNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.active = True
        self.lane_pose = LanePose()

        # distance from the stop line that we should stop
        self.stop_distance = 0.22
        # minimum number of red segments that we should detect to estimate a stop
        self.min_segs = 2
        # If y value of detected red line is smaller than max_y
        # we will not set at_stop_line true.
        self.max_y = 0.2

        # Subscribers
        self.sub_switch = rospy.Subscriber(
            "~switch",
            BoolStamped,
            self.cbSwitch
        )
        self.sub_segs = rospy.Subscriber(
            "/%s/line_detector_node/segment_list" % self.veh_name,
            SegmentList,
            self.processSegments
        )
        self.sub_lane = rospy.Subscriber(
            "/%s/lane_filter_node/lane_pose" % self.veh_name,
            LanePose,
            self.processLanePose
        )

        # Publishers
        self.pub_at_stop_line = rospy.Publisher(
            "~at_stop_line",
            BoolStamped,
            queue_size=1
        )

        rospy.loginfo('Initialized StopParkingDetectionNode')


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data


    def processLanePose(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg


    def processSegments(self, segment_list_msg):
        if not self.active:
            return

        good_seg_count = 0
        stop_line_x_accumulator = 0.0
        stop_line_y_accumulator = 0.0
        for segment in segment_list_msg.segments:
            if segment.color != segment.RED:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0: # the point is behind us
                continue

            p1_lane = self.to_lane_frame(segment.points[0])
            p2_lane = self.to_lane_frame(segment.points[1])
            avg_x = 0.5*(p1_lane[0] + p2_lane[0])
            avg_y = 0.5*(p1_lane[1] + p2_lane[1])
            stop_line_x_accumulator += avg_x
            stop_line_y_accumulator += avg_y # TODO output covariance and not just mean
            good_seg_count += 1.0

        if (good_seg_count < self.min_segs):
            msg = BoolStamped()
            msg.header.stamp = segment_list_msg.header.stamp
            msg.data = False
            self.pub_at_stop_line.publish(msg)
            return

        stop_line_point = Point()
        stop_line_point.x = stop_line_x_accumulator/good_seg_count
        stop_line_point.y = stop_line_y_accumulator/good_seg_count
        at_stop_line = stop_line_point.x < self.stop_distance and \
            math.fabs(stop_line_point.y) < self.max_y

        if at_stop_line:
            msg = BoolStamped()
            msg.header.stamp = segment_list_msg.header.stamp
            msg.data = True
            self.pub_at_stop_line.publish(msg)


    def to_lane_frame(self, point):
        p_homo = np.array([point.x,point.y,1])
        phi = self.lane_pose.phi
        d   = self.lane_pose.d
        T = np.array([[math.cos(phi), -math.sin(phi), 0],
                      [math.sin(phi), math.cos(phi) , d],
                      [0,0,1]])
        p_new_homo = T.dot(p_homo)
        p_new = p_new_homo[0:2]
        return p_new


if __name__ == '__main__':
    # Initialize the node
    stop_parking_detection = StopParkingDetectionNode(
        node_name='stop_parking_detection'
    )
    # Keep it spinning to keep the node alive
    rospy.spin()
