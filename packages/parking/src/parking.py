#!/usr/bin/env python

import os
import rospy
import time
from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped
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
        self.state = SEARCHING#IS_PARKED#ENTERING_PARKING_LOT
        self.red_line_counter = 0

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
        self.turn_direction_pub = rospy.Publisher(
            '~/%s/parking/turn_direction' % self.veh_name,
            String,
            queue_size=1
        )

        self.back_exit_pub = rospy.Publisher(
            '~/%s/parking/back_exit' % self.veh_name,
            BoolStamped,
            queue_size=1
        )

        # Subscribers
        self.free_parking_sub = rospy.Subscriber(
            '/%s/parking/free_parking' % self.veh_name,
            BoolStamped,
            self.cbParkingFree,
            queue_size=1
        )
        self.stop_parking_sub = rospy.Subscriber(
            '/%s/parking/white_line' % self.veh_name,
            BoolStamped,
            self.cbStopParking,
            queue_size=1
        )
        self.red_line_sub = rospy.Subscriber(
            '/%s/red_line' % self.veh_name,
            BoolStamped,
            self.cbRedLine,
            queue_size=1
        )

        # start again without rerunning

        """self.red_line_sub = rospy.Subscriber(
            '/%s/parking/start_from' % self.veh_name,
            BoolStamped,
            self.cdRestart,
            queue_size=1
        )"""

        #self.pauseOperations(5)
        #self.transitionToNextState()
        rospy.loginfo('[%s] Initialized' % self.node_name)

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """

    """def cdRestart(self,msg):
        self.state = SEARCHING
    """
    def cbParkingFree(self, msg):

        found_free_parking_spot = msg.data == True
        # We only care about finding a free spot if we're searching
        if self.state == EXITING_PARKING_SPOT:
            if found_free_parking_spot:
                rospy.loginfo('[%s] Exited parking spot!' % self.node_name)
                self.pauseOperations(2)
                self.turn('left')
                self.transitionToNextState()

        if self.state != SEARCHING:
            return

        if found_free_parking_spot:
            rospy.loginfo('[%s] Found a free parking spot!' % self.node_name)
            self.transitionToNextState()


    def cbStopParking(self, msg):
        # We only care about stopping parking if we're currently parking
        if self.state != IS_PARKING:
            return

        should_stop_parking = msg.data == True
        if should_stop_parking:
            rospy.loginfo('[%s] Stop parking maneuver!' % self.node_name)
            self.transitionToNextState()


    def cbRedLine(self, msg):
        # We only care about turning at a red line if we're in certain states
        if self.state in [INACTIVE, IS_PARKED, IS_PARKING, EXITING_PARKING_SPOT]:
            return

        at_intersection = msg.data == True
        if at_intersection:
            rospy.loginfo('[%s] Detected intersection!' % self.node_name)
            self.pauseOperations(2)

            if self.state == ENTERING_PARKING_LOT:
                self.turn('right') # Turn right to enter parking area
                self.transitionToNextState() # Begin searching
            elif self.state == SEARCHING:
                if self.red_line_counter % 2 == 0:
                    # We are opposite the parking entrance/exit, so we
                    # continue straight with normal lane following.
                    # Pause for a few seconds to get past the red line
                    rospy.sleep(4)
                else:
                    # We are at the parking entrance/exit but still searching
                    # for parking. The Duckiebot would try to turn right,
                    # so we force it to go straight here
                    self.turn('straight')
                self.red_line_counter += 1 # Passed a red line on outer loop
            elif self.state == EXITING_PARKING_LOT:
                if self.red_line_counter == -1:
                    # Special flag of -1 means a right turn is a guaranteed exit
                    self.turn('right')
                if self.red_line_counter % 2 == 0:
                    # Even number of red lines seen after entering parking lot,
                    # so the Duckiebot parked in the first row. Go left to
                    # continue moving around the loop, eventually to the exit.
                    self.turn('left')
                    # Set to special value -1, next red line = guaranteed exit
                    self.red_line_counter = -1
                else:
                    # Odd number of red lines seen after entering parking lot,
                    # so the Duckiebot parked in the second row. Go straight
                    # to directly exit the parking lot.
                    self.turn('straight')

    """
    #############################
    ###### HELPER FUNCTIONS #####
    #############################
    """

    def transitionToNextState(self):
        next_state = self.state + 1
        if next_state not in ALL_STATES:
            next_state = INACTIVE
        self.state = next_state

        if next_state == SEARCHING:
            self.startNormalLaneFollowing()

        elif next_state == IS_PARKING:
            self.updateTopCutoff(80) # Cut out blue lines from other parking spots
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.updateDoffset(0.18) # Follow the left of the blue lane
            self.setLEDs('blink') # Blink LEDs to indicate we are parking now
            self.pauseOperations(2) # Pause for 2 sec before continuing

        elif next_state == IS_PARKED:
             # No longer need to cut off top of image
            rospy.loginfo('[%s] IS_PARKED' % (self.node_name))
            self.setLEDs('off') # Turn off LEDs while parked
            self.pauseOperations(5) # Stay in the parking spot a certain time
            self.transitionToNextState() # Start exiting the parking spot

        elif next_state == EXITING_PARKING_SPOT:
            rospy.loginfo('[%s] EXITING_PARKING_SPOT' % (self.node_name))
            self.startBackLaneFollowing()
            #self.turn('right') # Turn right to exit the parking spot
            #self.transitionToNextState() # Start exiting the parking lot

        elif next_state == EXITING_PARKING_LOT:
            rospy.loginfo('[%s] EXITING_PARKING_LOT' % (self.node_name))
            self.pauseOperations(5)
            self.startNormalLaneFollowing()

    def publish_exit(self,value):
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = value
        self.back_exit_pub.publish(msg)

    def pauseOperations(self, num_sec):
        tup = (self.node_name, num_sec)
        rospy.loginfo('[%s] Attempting to pause for %.1f seconds' % tup)
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def startNormalLaneFollowing(self):
        self.setLEDs('white') # Set LEDs to white (normal operation)
        self.updateDoffset(0.0) # d_offset=0 for normal lane following
        self.updateTopCutoff() # No top cutoff for normal lane following
        self.updateLaneFilterColor('yellow') # Follow yellow lines (normal)
        self.publish_exit(False)
        self.turn('none') # No special maneuvers

    def startBackLaneFollowing(self):
        self.updateTopCutoff(50)
        self.updateDoffset(0.118)
        self.setLEDs('red') # Set LEDs to red to indicate leaving parking
        self.updateLaneFilterColor('blue')
        self.publish_exit(True)
        self.turn('none') # No special maneuvers

    def turn(self, direction, duration=1.5):
        if direction not in ['straight', 'right', 'left', 'none']:
            tup = (self.node_name, direction)
            rospy.logerr('[%s] Invalid direction: %s' % tup)
            return

        def _deactivate(e):
            no_turn = String()
            no_turn.data = 'none'
            self.turn_direction_pub.publish(no_turn)
            tup = (self.node_name, direction)
            rospy.loginfo('[%s] Finished turning %s' % tup)

        if direction == 'none':
            _deactivate(None)
            return

        rospy.loginfo('[%s] Turning %s' % (self.node_name, direction))
        activate_turn = String()
        activate_turn.data = direction
        self.turn_direction_pub.publish(activate_turn)
        rospy.sleep(duration)
        _deactivate(None)


    def updateDoffset(self, new_offset):
        tup = (self.node_name, new_offset)
        rospy.loginfo('[%s] Publishing new d_offset: %.3f' % tup)
        self.d_offset_pub.publish(new_offset)


    def updateLaneFilterColor(self, desired_color):
        # Desired_color should be one of 'yellow', 'green', 'blue'
        tup = (self.node_name, desired_color)
        rospy.loginfo('[%s] Publishing new color for lane_filter: %s' % tup)
        self.lane_filter_color_pub.publish(desired_color)


    def updateTopCutoff(self, cutoff=40):
        # Cutoff (don't examine) the top section of the image for line detector
        tup = (self.node_name, cutoff)
        rospy.loginfo('[%s] Setting line detector top_cutoff=%d' % tup)
        param_name = '/%s/line_detector_node/top_cutoff' % self.veh_name
        rospy.set_param(param_name, cutoff)


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
