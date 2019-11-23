#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped


# States
ENTERING_PARKING_LOT = 1
SEARCHING = 2
IS_PARKING = 3
IS_PARKED = 4
EXITING_PARKING_SPOT = 5
EXITING_PARKING_LOT = 6


class ParkingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ParkingNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.node_name = 'ParkingNode'
        self.state = SEARCHING

        offset_mode = 2
        if offset_mode == 0:
            self.offset = 0.0
        elif offset_mode == 1:
            self.offset = 0.2175    # lane distance
        elif offset_mode == 2:
            self.offset = 0.115     # middle of the road
        else:
            self.offset = 0.0

        # Publishers
        self.d_offset_pub = rospy.Publisher(
            'lane_controller_node/doffset',
            Float64,
            queue_size=1
        )

        # Subscribers
        self.vehicle_detection_sub = rospy.Subscriber(
            '/%s/vehicle_detection_node/detection' % self.veh_name,
            BoolStamped,
            self.cbOvertake,
            queue_size=1
        )
        self.free_parking_sub = rospy.Subscriber(
            '/%s/free_parking' % self.veh_name,
            BoolStamped,
            self.parkingFreeCb,
            queue_size=1
        )

        rospy.loginfo('%s initialized' % self.node_name)


    def parkingFreeCb(self, msg):
        # We only care about finding a free spot if we're searching
        if self.state != SEARCHING:
            return

        found_free_parking_spot = msg.data == True
        if found_free_parking_spot:
            rospy.loginfo('%s found a free parking spot!' % self.node_name)
            self.transitionToNextState()


    def transitionToNextState(self):
        current_state = self.state
        next_state = self.state + 1

        if current_state == SEARCHING and next_state == IS_PARKING:
            rospy.set_param('/parking/lane_color', 'blue')
            self.updateDoffset(0)

        self.state = next_state


    def updateDoffset(self, new_offset):
        rospy.loginfo('%s publishing new d_offset: %f' % (self.node_name, new_offset))
        self.d_offset_pub.publish(new_offset)


    def cbOvertake(self, msg):
        if msg.data:
            rospy.loginfo('[%s] detected vehicle' % self.node_name)
            # self.offset =  0.2175/4*1 #write
            # rospy.sleep(0.5)
            # self.offset =  0.2175/4*2 #write
            # rospy.sleep(0.5)
            # self.offset =  0.2175/4*3 #write
            # rospy.sleep(0.5)
            # self.offset =  0.2175/4*4 #write
            # rospy.sleep(7.)
            # print "going back to the right lane"
            # self.offset =  0.2175/4*3 #write
            # rospy.sleep(0.5)
            # self.offset =  0.2175/4*2 #write
            # rospy.sleep(0.5)
            # self.offset =  0.2175/4*1 #write
            # rospy.sleep(0.5)
            self.offset =  0.0


if __name__ == '__main__':
    # create the node
    node = ParkingNode(node_name='parking_node')
    # keep spinning
    rospy.spin()
