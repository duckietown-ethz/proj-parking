#!/usr/bin/env python

import os
import rospy
import time
from duckietown import DTROS
from std_msgs.msg import String, Float64, Int16
from duckietown_msgs.msg import (
    BoolStamped,
    Twist2DStamped,
    WheelsCmdStamped
)
#from duckietown_msgs.srv import ChangePattern

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
WAITING_FOR_EXITING_DBOT = 7

ALL_STATES = [
    INACTIVE,
    ENTERING_PARKING_LOT,
    SEARCHING,
    IS_PARKING,
    IS_PARKED,
    EXITING_PARKING_SPOT,
    EXITING_PARKING_LOT,
    WAITING_FOR_EXITING_DBOT
]

speed = 1.0


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
        self.state = IS_PARKED#SEARCHING

        # Services
        #self.set_led_pattern = rospy.ServiceProxy(
        #    '/%s/led_emitter_node/set_pattern' % self.veh_name,
        #    ChangePattern
        #)

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
        
        self.pub_wheel_cmd = rospy.Publisher(
            '~/%s/wheels_driver_node/wheels_cmd' % self.veh_name,
            WheelsCmdStamped,
            queue_size=10
        )
        self.pub_car_cmd = rospy.Publisher(
            "~/%s/lane_controller_node/car_cmd" % self.veh_name,
            Twist2DStamped,
            queue_size=10
        )
        self.pub_reverse = rospy.Publisher(
            '~/%s/parking/reverse' % self.veh_name,
            BoolStamped,
            queue_size=1
        )

        # Subscribers
        self.free_parking_sub = rospy.Subscriber(
            '/%s/free_parking' % self.veh_name,
            BoolStamped,
            self.cbParkingFree,
            queue_size=1
        )
        self.stop_parking_sub = rospy.Subscriber(
            '/%s/red_line' % self.veh_name,
            BoolStamped,
            self.cbStopParking,
            queue_size=1
        )
        """self.sub_mode = rospy.Subscriber(
            "~fsm_mode", 
            FSMState, 
            self.cbModeFSM, 
            queue_size = 1
        )"""
        
        # Set the current finite state machine state to PARKING
        #self.fsm_mode = self.setupParameter("~initial_mode","PARKING")


        self.transitionToNextState()
        rospy.loginfo('[%s] Initialized' % self.node_name)

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """
    """def cbModeFSM(self, data):
        self.fsm_mode = data.state
        if self.fsm_mode == self.reset_mode:
            self.actions = []
            rospy.wait_for_service('graph_search')
            graph_search = rospy.ServiceProxy('graph_search', GraphSearch)
            graph_search('0', '0')
        elif self.localization_mode != "none" and self.fsm_mode == self.localization_mode:
            self.pubLocalized()
        self.dispatcher()"""

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

        """######################################
        ### Leave/Stay in the parking area ###
        ######################################
        if self.state == EXITING_PARKING_LOT:
            # turn right as in the intersection
            self.pub_turn_type.publish(2)# this mystical number correspond to the array ordering in open_loop_intersection_control_node
        if self.state != IS_PARKING:
            # turn left as in the intersection
            self.pub_turn_type.publish(1)# this mystical number correspond to the array ordering in open_loop_intersection_control_node
        
        """
        
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
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def transitionToNextState(self):
        next_state = self.state + 1
        if next_state not in ALL_STATES:
            next_state = INACTIVE
        self.state = next_state

        if next_state == IS_PARKING:
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.updateDoffset(0.18) # Follow the left of the blue lane
            self.setLEDs('blink') # Blink LEDs to indicate we are parking now
            self.pauseOperations(2) # Pause for 2 sec before continuing

        elif next_state == IS_PARKED:
            self.setLEDs('off') # Turn off LEDs while parked
            self.pauseOperations(5) # Stay in the parking spot a certain time
            self.transitionToNextState() # Start exiting the parking spot

        elif next_state == EXITING_PARKING_SPOT:
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.setLEDs('red') # Set LEDs to red to indicate leaving parking
            self.pauseOperations(3) # Allow time for others to detect LEDs
            self.driveBackwards() # Begin maneuver to exit parking spot

        elif next_state == EXITING_PARKING_LOT:
            self.setLEDs('white') # Set LEDs to white (normal operation)
            self.updateDoffset(0.0) # d_offset=0 for normal lane following
            self.updateLaneFilterColor('yellow') # Follow yellow lines (normal)

        else:
            pass # TODO - handle other states


    def pauseOperations(self, num_sec):
        rospy.loginfo('[%s] Attempting to pause for %.1f seconds' % (self.node_name, num_sec))
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def driveBackwards(self):
        reverse = BoolStamped()
        reverse.header.stamp = rospy.Time.now()
        timeout = time.time() + 1.5 # 1.5 seconds from now
        while True:
            reverse.data = True
            self.pub_reverse.publish(reverse)
            if time.time() > timeout:
                reverse.data = False
                self.pub_reverse.publish(reverse)
                break
        self.transitionToNextState() # Reversal maneuver completed -> next state


    def updateDoffset(self, new_offset):
        rospy.loginfo('[%s] Publishing new d_offset: %f' % (self.node_name, new_offset))
        self.d_offset_pub.publish(new_offset)


    def updateLaneFilterColor(self, desired_color):
        # desired_color should be one of 'yellow', 'green', 'blue'
        tup = (self.node_name, desired_color)
        rospy.loginfo('[%s] Publishing new color for lane_filter: %s' % tup)
        self.lane_filter_color_pub.publish(desired_color)


    def setLEDs(self, pattern):
        msg = None
        if pattern == 'white':
            msg = 'WHITE'
        elif pattern == 'red':
            msg = 'RED'
        elif pattern == 'off':
            msg = 'LIGHT_OFF'
        elif pattern == 'blink':
            msg = 'CAR_SIGNAL_SACRIFICE_FOR_PRIORITY'

        if msg is not None:
            rospy.loginfo('[%s] Settings LEDs to %s' % (self.node_name, msg))
            pattern = String()
            pattern.data = msg
            #self.set_led_pattern(pattern)


if __name__ == '__main__':
    # create the node
    node = ParkingNode(node_name='parking_node')
    # keep spinning
    rospy.spin()
