#!/usr/bin/env python

import os
import rospy
import time
from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.srv import ChangePattern


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
        # Initialize the DTROS parent class
        super(ParkingNode, self).__init__(node_name=node_name)

        self.veh_name = rospy.get_namespace().strip("/")
        self.node_name = rospy.get_name()
        self.state = ENTERING_PARKING_LOT

        # Services
        self.set_led_pattern = rospy.ServiceProxy(
            '/%s/led_emitter_node/set_pattern' % self.veh_name,
            ChangePattern
        )

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
        self.reverse_pub = rospy.Publisher(
            '~/%s/parking/reverse' % self.veh_name,
            BoolStamped,
            queue_size=1
        )
        self.joystick_control_pub = rospy.Publisher(
            'joy_mapper_node/joystick_override',
            BoolStamped,
            queue_size=1
        )

        # Subscribers
        self.parking_spot_detection_sub = rospy.Subscriber(
            '/%s/parking/free_parking' % self.veh_name,
            BoolStamped,
            self.cbParkingSpotDetected,
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
        self.pink_led_sub = rospy.Subscriber(
            '/%s/parking/pink_leds' % self.veh_name,
            BoolStamped,
            self.cbPinkLEDDetection,
            queue_size=1
        )
        
        # start again without re-running
        self.restart_sub = rospy.Subscriber(
            '/%s/parking/start_from' % self.veh_name,
            BoolStamped,
            self.cbRestart,
            queue_size=1
        )

        rospy.loginfo('[%s] Initialized.' % self.node_name)

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """

    def cbRestart(self, msg):
        if msg.data:
            rospy.loginfo('[%s] Resetting state to SEARCHING' % self.node_name)
            self.state = SEARCHING


    def cbParkingSpotDetected(self, msg):
        # We only care about finding a spot if we're searching or exiting
        if self.state not in [SEARCHING, EXITING_PARKING_SPOT]:
            return

        found_free_parking_spot = (msg.data == True)
        if not found_free_parking_spot:
            return

        if self.state == SEARCHING:
            rospy.loginfo('[%s] Found a free parking spot!' % self.node_name)
        elif self.state == EXITING_PARKING_SPOT:
            rospy.loginfo('[%s] Exited parking spot!' % self.node_name)

        self.transitionToNextState()


    def cbStopParking(self, msg):
        # We only care about stopping parking if we're currently parking
        if self.state != IS_PARKING:
            return

        should_stop_parking = (msg.data == True)
        if should_stop_parking:
            rospy.loginfo('[%s] Stop parking maneuver!' % self.node_name)
            self.transitionToNextState()


    def cbRedLine(self, msg):
        # We only care about turning at a red line if we're in certain states
        if self.state not in [SEARCHING, ENTERING_PARKING_LOT, EXITING_PARKING_LOT]:
            return

        at_intersection = msg.data == True
        if at_intersection:
            rospy.loginfo('[%s] Detected intersection!' % self.node_name)
            self.pauseOperations(2)

            if self.state == ENTERING_PARKING_LOT:
                self.turn('right') # Turn right to enter parking area
                self.transitionToNextState() # Begin searching
            elif self.state == SEARCHING:
                self.turn('straight', 2.0) # Go straight for some time
            elif self.state == EXITING_PARKING_LOT:
                self.turn('right') # Turn right to exit the parking lot


    def cbPinkLEDDetection(self, msg):
        # We only stop for pink LEDs in certain states
        if self.state not in [SEARCHING, ENTERING_PARKING_LOT, EXITING_PARKING_LOT]:
            return

        sees_pink_leds = (msg.data == True)
        if sees_pink_leds:
            rospy.loginfo('[%s] Detected Duckiebot with pink LEDs!' % self.node_name)
            self.pauseOperations(10) # Pause for some time till danger is gone

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
            rospy.loginfo('[%s] SEARCHING' % self.node_name)
            self.startNormalLaneFollowing()

        elif next_state == IS_PARKING:
            rospy.loginfo('[%s] IS_PARKING' % self.node_name)
            # self.updateGain(0.8) # Updates the gain if needed
            self.updateTopCutoff(80) # Cut out blue lines from other parking spots
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.updateDoffset(0.18) # Follow the left of the blue lane
            self.setLEDs('blink') # Blink LEDs to indicate we are parking now
            self.pauseOperations(2) # Pause for 2 sec before continuing

        elif next_state == IS_PARKED:
            rospy.loginfo('[%s] IS_PARKED' % self.node_name)
            self.setLEDs('off') # Turn off LEDs while parked
            self.pauseOperations(5) # Stay in the parking spot a certain time
            self.transitionToNextState() # Start exiting the parking spot

        elif next_state == EXITING_PARKING_SPOT:
            rospy.loginfo('[%s] EXITING_PARKING_SPOT' % self.node_name)
            self.setLEDs('red') # Set LEDs to red to indicate leaving parking
            self.pauseOperations(3) # Allow time for others to detect LEDs
            self.startBackwardsLaneFollowing() # Backwards wheel commands

        elif next_state == EXITING_PARKING_LOT:
            rospy.loginfo('[%s] EXITING_PARKING_LOT' % self.node_name)
            self.pauseOperations(2) # Pause for a few seconds
            self.startNormalLaneFollowing() # Resume normal lane following
            self.pauseOperations(2) # Pause for a few more seconds


    def pauseOperations(self, num_sec):
        tup = (self.node_name, num_sec)
        rospy.loginfo('[%s] Attempting to pause for %.1f seconds' % tup)
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def updateDoffset(self, new_offset):
        tup = (self.node_name, new_offset)
        rospy.loginfo('[%s] Publishing new d_offset: %.3f' % tup)
        self.d_offset_pub.publish(new_offset)


    def updateTopCutoff(self, cutoff=40):
        # Cutoff (don't examine) the top section of the image for line detector
        tup = (self.node_name, cutoff)
        rospy.loginfo('[%s] Setting line detector top_cutoff=%d' % tup)
        param_name = '/%s/line_detector_node/top_cutoff' % self.veh_name
        rospy.set_param(param_name, cutoff)


    def updateGain(self, gain=0.8):
        # Update the gain of the kinematics package
        gain_update = (self.node_name, gain)
        rospy.loginfo('[%s] Setting kinematic control gain=%.2f' % gain_update)
        param_name = '/%s/kinematics_node/gain' % self.veh_name
        rospy.set_param(param_name, gain)


    def updateLaneFilterColor(self, desired_color):
        # desired_color should be one of 'yellow', 'green', 'blue'
        tup = (self.node_name, desired_color)
        rospy.loginfo('[%s] Publishing new color for lane_filter: %s' % tup)
        self.lane_filter_color_pub.publish(desired_color)


    def toggleReversal(self, reverse):
        tup = (self.node_name, reverse)
        rospy.loginfo('[%s] Settings reversal to %s' % tup)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = reverse
        self.reverse_pub.publish(msg)


    def startBackwardsLaneFollowing(self):
        self.updateTopCutoff(50) # Cut off upper image
        self.updateDoffset(0.118) # Follow center of lane
        self.updateLaneFilterColor('blue') # Follow blue lines
        self.toggleReversal(reverse=True) # Backwards wheel commands
        self.turn('none') # No special turning maneuvers


    def startNormalLaneFollowing(self, restart=True):
        self.toggleReversal(reverse=False) # No more reverse control
        self.setLEDs('white') # Set LEDs to white (normal operation)
        self.updateDoffset(0) # d_offset=0 for normal lane following
        self.updateTopCutoff() # Default top cutoff for normal lane following
        self.updateLaneFilterColor('yellow') # Follow yellow lines (normal)
        self.turn('none') # No special turning maneuvers
        if restart:
            self.restartLaneFollowing() # Switch FSM off and on again


    def restartLaneFollowing(self):
        override = BoolStamped()
        override.header.stamp = rospy.Time.now()
        override.data = True # Activate joystick control
        self.joystick_control_pub.publish(override)
        rospy.sleep(1) # Allow time for FSM state transition
        override.data = False # Deactivate joystick --> activate lane following
        self.joystick_control_pub.publish(override)


    def turn(self, direction, duration=1.5):
        if direction not in ['straight', 'right', 'left', 'none']:
            tup = (self.node_name, direction)
            rospy.logerr('[%s] Invalid direction: %s' % tup)
            return

        # Helper function to STOP any special turning maneuvers
        def _deactivate():
            no_turn = String()
            no_turn.data = 'none'
            self.turn_direction_pub.publish(no_turn)
            tup = (self.node_name, direction)
            rospy.loginfo('[%s] Finished turning %s' % tup)

        if direction == 'none':
            _deactivate()
            return

        rospy.loginfo('[%s] Turning %s' % (self.node_name, direction))
        activate_turn = String()
        activate_turn.data = direction
        self.turn_direction_pub.publish(activate_turn)
        rospy.sleep(duration)
        _deactivate()


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
            self.set_led_pattern(pattern)


if __name__ == '__main__':
    # create the node
    node = ParkingNode(node_name='parking_node')
    # keep spinning
    rospy.spin()
