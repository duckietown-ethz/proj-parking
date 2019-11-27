#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped, Twist2DStamped
# from duckietown_msgs.srv import ChangePattern

"""
#############################
########### STATES ##########
#############################
"""

INACTIVE = 0
ENTERING_PARKING_LOT = 1
SEARCHING = 2
IS_PARKING = 3
IS_PARKED = 4
EXITING_PARKING_SPOT = 5
EXITING_PARKING_LOT = 6

ALL_STATES = [
    INACTIVE,
    ENTERING_PARKING_LOT,
    SEARCHING,
    IS_PARKING,
    IS_PARKED,
    EXITING_PARKING_SPOT,
    EXITING_PARKING_LOT
]


class ParkingNode(DTROS):

    """
    #############################
    ####### INITIALIZATION ######
    #############################
    """

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(ParkingNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.node_name = 'ParkingNode'
        self.state = SEARCHING

        # Services
        # self.set_led_pattern = rospy.ServiceProxy(
        #     '/%s/led_emitter_node/set_pattern' % self.veh_name,
        #     ChangePattern
        # )

        # Publishers
        self.d_offset_pub = rospy.Publisher(
            'lane_controller_node/doffset',
            Float64,
            queue_size=1
        )
        self.lane_filter_color_pub = rospy.Publisher(
            '/parking/lane_color',
            String,
            queue_size=1
        )
        self.pause_lane_control_pub = rospy.Publisher(
            'lane_controller_node/pause',
            Float64,
            queue_size=1
        )

        # Subscribers
        self.vehicle_avoidance_wheel_cmd_sub = rospy.Subscriber(
            '/%s/vehicle_avoidance_control_node/car_cmd' % self.veh_name,
            Twist2DStamped,
            self.cbVehicleAvoidanceControl,
            queue_size=1
        )
        self.free_parking_sub = rospy.Subscriber(
            '/%s/free_parking' % self.veh_name,
            BoolStamped,
            self.cbParkingFree,
            queue_size=1
        )
        self.stop_parking_sub = rospy.Subscriber(
            '/%s/parking_stop/at_stop_line' % self.veh_name,
            BoolStamped,
            self.cbStopParking,
            queue_size=1
        )

        rospy.loginfo('[%s] Initialized' % self.node_name)

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """

    def cbParkingFree(self, msg):
        # We only care about finding a free spot if we're searching
        if self.state != SEARCHING:
            return

        found_free_parking_spot = msg.data == True
        if found_free_parking_spot:
            rospy.loginfo('[%s] Found a free parking spot!' % self.node_name)
            self.transitionToNextState()


    def cbStopParking(self, msg):
        rospy.loginfo('parking.py stop_parking=%s' % str(msg.data))

        # We only care about stopping parking if we're currently parking
        if self.state != IS_PARKING:
            return

        should_stop_parking = msg.data == True
        if should_stop_parking:
            rospy.loginfo('[%s] Stop parking maneuver!' % self.node_name)
            self.transitionToNextState()


    def cbVehicleAvoidanceControl(self, twist):
        # We will only stop for another Duckiebot in certain states
        if self.state not in [SEARCHING, IS_PARKED]:
            return

        if twist.v == 0 and twist.omega == 0:
            rospy.loginfo('[%s] Vehicle avoidance wants to stop!' % self.node_name)
            # self.pauseOperations(10)

    """
    #############################
    ###### HELPER FUNCTIONS #####
    #############################
    """

    def transitionToNextState(self):
        current_state = self.state
        next_state = self.state + 1

        if current_state == SEARCHING and next_state == IS_PARKING:
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.updateDoffset(0.22) # Follow the center of the lane
            self.blinkLEDs() # Blink LEDs to indicate we are parking now
            self.pauseOperations(10) # Pause for 10 sec before continuing
        elif current_state == IS_PARKING:
            self.pauseOperations(30) # Stay in the parking spot 30 seconds
        else:
            pass # TODO - handle other states

        if next_state in ALL_STATES:
            self.state = next_state
        else:
            self.state = INACTIVE


    def pauseOperations(self, num_sec):
        rospy.loginfo('[%s] Attempting to pause for %f seconds' % (self.node_name, num_sec))
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def updateDoffset(self, new_offset):
        rospy.loginfo('[%s] Publishing new d_offset: %f' % (self.node_name, new_offset))
        self.d_offset_pub.publish(new_offset)


    def updateLaneFilterColor(self, desired_color):
        # desired_color should be one of 'yellow', 'green', 'blue'
        rospy.loginfo('[%s] Publishing new color for lane_filter: %s' % (self.node_name, desired_color))
        self.lane_filter_color_pub.publish(desired_color)


    def blinkLEDs(self):
        rospy.loginfo('[%s] Blinking LEDs' % self.node_name)
        # pattern = ChangePattern()
        # pattern.pattern_name = 'CAR_SIGNAL_SACRIFICE_FOR_PRIORITY'
        # self.set_led_pattern(pattern)


if __name__ == '__main__':
    # create the node
    node = ParkingNode(node_name='parking_node')
    # keep spinning
    rospy.spin()
