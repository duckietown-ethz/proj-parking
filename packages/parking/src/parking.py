#!/usr/bin/env python

"""
The parking node is the main control point for parking.
It runs simultaneously with lane following and has an
internal state which is used to configure other nodes
(e.g. lane controller and line filter) in various ways,
depending on what the Duckiebot is doing in the parking area.
"""

import os
import rospy
import time
import random

from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped, LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern


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
        self.state = INACTIVE # Initially, the node is inactive
        self.at_red_line = False
        self.blob_detected = False
        self.ignore_red_lines = False
        # Time slots for parked Duckiebots
        self.time_slot_exiting = 20
        # Time slots for Duckiebots searching for parking
        self.time_slot_searching = 5

        ###### Services ######
        
        # Service to change LEDs to a custom pattern
        self.set_custom_led_pattern = rospy.ServiceProxy(
            '/%s/led_emitter_node/set_custom_pattern' % self.veh_name,
            SetCustomLEDPattern
        )

        ###### Publishers ######
        
        # Publisher to deactivate the parking FSM mode
        self.stop_parking_fsm_mode = rospy.Publisher(
            '/%s/parking_off' % self.veh_name,
            BoolStamped,
            queue_size=1
        )
        # Dynamically change the d_offset of lane control
        self.d_offset_pub = rospy.Publisher(
            'lane_controller_node/doffset',
            Float64,
            queue_size=1
        )
        # Dynamically change the color for the lane filter
        self.lane_filter_color_pub = rospy.Publisher(
            '/parking/lane_color',
            String,
            queue_size=1
        )
        # Pause the lane controller
        self.pause_lane_control_pub = rospy.Publisher(
            'lane_controller_node/pause',
            Float64,
            queue_size=1
        )
        # Override the lane controller with a manual movement
        self.lane_control_override_pub = rospy.Publisher(
            '~/%s/parking/lane_control_override' % self.veh_name,
            String,
            queue_size=1
        )
        # Make the lane controller go backwards if True, else forwards
        self.reverse_pub = rospy.Publisher(
            '~/%s/parking/reverse' % self.veh_name,
            BoolStamped,
            queue_size=1
        )
        # Publish True if we should detect LEDs on the right-half of the
        # image, otherwise use the left-half of the image if False
        self.led_detection_right_pub = rospy.Publisher(
            '~/%s/parking/led_detection_right' % self.veh_name,
            BoolStamped,
            queue_size=1
        )

        ###### Subscribers ######
        
        # Subscriber to FSM switch (activate or deactivate the node)
        self.switch_sub = rospy.Subscriber(
            '~switch',
            BoolStamped,
            self.cbSwitch,
            queue_size=1
        )
        # Returns True if the Duckiebot detected a parking spot
        self.parking_spot_detection_sub = rospy.Subscriber(
            '/%s/parking/free_parking' % self.veh_name,
            BoolStamped,
            self.cbParkingSpotDetected,
            queue_size=1
        )
        # Returns True if the Duckiebot should stop parking maneuver
        self.stop_parking_sub = rospy.Subscriber(
            '/%s/parking/white_line' % self.veh_name,
            BoolStamped,
            self.cbStopParking,
            queue_size=1
        )
        # Returns True when a red line is detected, False otherwise
        self.red_line_sub = rospy.Subscriber(
            '/%s/red_line' % self.veh_name,
            BoolStamped,
            self.cbRedLine,
            queue_size=1
        )
        # Returns True when a red LED is detected, False otherwise
        self.red_led_sub = rospy.Subscriber(
            '/%s/parking/red_led' % self.veh_name,
            BoolStamped,
            self.cbRedLED,
            queue_size=1
        )
        # Start again without re-running
        self.restart_sub = rospy.Subscriber(
            '/%s/parking/start_from' % self.veh_name,
            BoolStamped,
            self.cbRestart,
            queue_size=1
        )
        # When this subscriber receives `True` and the Duckiebot is
        # parked, the Duckiebot will exit the parking spot
        self.exit_parking_spot_sub = rospy.Subscriber(
            '~/%s/parking/time_exiting_parking_spot' % self.veh_name,
            BoolStamped,
            self.cbLeaveParkingSpot,
            queue_size=1
        )

        self.log('Initialized.')

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """

    def cbLeaveParkingSpot(self, msg):
        # When the Duckiebot is parked and you want it to exit,
        # you can send `True` to exit_parking_spot_sub and the
        # Duckiebot will shortly thereafter exit the parking spot
        if msg.data and self.state == IS_PARKED:
            self.transitionToNextState()


    def cbRestart(self, msg):
        # Useful for debugging: this enables us to reset the state
        if msg.data:
            self.log('Resetting state to ENTERING_PARKING_LOT')
            self.state = INACTIVE
            self.at_red_line = False
            self.blob_detected = False
            self.ignore_red_lines = False
            self.transitionToNextState() # Transition to ENTERING_PARKING_LOT


    def cbSwitch(self, fsm_switch_msg):
        # Finite State Machine switches the parking node on or off
        was_inactive = (self.state == INACTIVE)
        becoming_active = fsm_switch_msg.data

        if was_inactive and becoming_active:
            # Transition to ENTERING_PARKING_LOT after a short delay
            cb = lambda e: self.transitionToNextState()
            rospy.Timer(rospy.Duration.from_sec(0.6), cb, oneshot=True)
            rospy.sleep(0.5)
        elif not becoming_active:
            self.state = INACTIVE

        self.log('active: %s' % becoming_active)


    def cbParkingSpotDetected(self, msg):
        # We only care about finding a spot if we're searching for a
        # parking spot or exiting a parking spot
        if self.state not in [SEARCHING, EXITING_PARKING_SPOT]:
            return

        found_free_parking_spot = (msg.data == True)
        if not found_free_parking_spot:
            return

        if self.state == SEARCHING:
            self.log('Found a free parking spot!')
        elif self.state == EXITING_PARKING_SPOT:
            self.log('Exited parking spot!')

        self.transitionToNextState()


    def cbStopParking(self, msg):
        # We only care about stopping parking if we're currently parking
        if self.state != IS_PARKING:
            return

        should_stop_parking = (msg.data == True)
        if should_stop_parking:
            self.log('Stopping parking maneuver!')
            self.transitionToNextState()


    def cbRedLine(self, msg):
        # We only care about turning at a red line if we're in certain states
        if self.state not in [SEARCHING, ENTERING_PARKING_LOT, EXITING_PARKING_LOT]:
            return

        if self.ignore_red_lines:
            return

        at_intersection = (msg.data == True)
        if at_intersection:
            self.at_red_line = True
            self.log('Detected intersection!')
            self.manualLaneControl('stop') # Stop indefinitely (no timeout)

            if self.state == EXITING_PARKING_LOT:
                # Pause for a few seconds at the stop line
                self.pauseOperations(3.0)
                # No need to look for another red line in the near future
                self.pauseRedLineDetection()
                # Short manual right turn
                self.manualLaneControl('right', duration=1.5)
                # Exit parking mode in the FSM
                self.stopParkingModeFSM()
                return

            self.setLEDs('red') # Set LEDs to indicate we are at intersection
            self.pauseOperations(3)

            if self.state == ENTERING_PARKING_LOT:
                # At intersection; look on the left for Duckiebots going straight
                self.toggleLEDDetection(led_detection_right=False)

                while self.blob_detected:
                    self.log('At intersection and other duckie detected!')
                    self.setLEDs('switchedoff') # Turn off LEDs while waiting
                    self.pauseOperations(10)
                    self.blob_detected = False
                    self.setLEDs('red') # Set LEDs to indicate we're at intersection
                    self.pauseOperations(1)

                # No need to look for another red line in the near future
                self.pauseRedLineDetection()
                # Short manual right turn
                self.manualLaneControl('right', duration=1.5)
                # Begin searching
                self.transitionToNextState()

            elif self.state == SEARCHING:
                # At intersection while searching
                # Look on the right for a Duckiebot coming into parking area
                self.toggleLEDDetection(led_detection_right=True)

                while self.blob_detected:
                    self.log('At intersection and other duckie detected!')
                    self.setLEDs('switchedoff') # Turn off LEDs while waiting
                    self.pauseOperations(10)
                    self.blob_detected = False
                    self.setLEDs('red') # Set LEDs to indicate we are at intersection
                    self.pauseOperations(1)

                # Go straight at the intersection to continue searching
                self.manualLaneControl('straight', duration=2.5)
                # Look on the left for Duckiebots backing out of parking spots
                self.toggleLEDDetection(led_detection_right=False)

        self.at_red_line = False


    def cbRedLED(self, msg):
        # We only stop for LED detection in certain states
        if self.state not in [SEARCHING, ENTERING_PARKING_LOT, EXITING_PARKING_LOT]:
            return

        blob_detected = (msg.data == True)

        if not blob_detected:
            return

        self.blob_detected = blob_detected
        self.log('Detected Duckiebot with red LEDs!')

        if self.at_red_line or self.state == ENTERING_PARKING_LOT:
            return

        self.log('Detected Duckiebot with red LEDs (no intersection)!')
        self.setLEDs('red') # Set LEDs to indicate we saw a Duckie that wants to exit
        self.waitForRandomTime()
        self.setLEDs('switchedoff') # Set LEDs off while lane following

    """
    #############################
    ###### HELPER FUNCTIONS #####
    #############################
    """

    def log(self, message, type='info'):
        full_message = '[%s] %s' % (self.node_name, message)
        if type == 'info':
            rospy.loginfo(full_message)
        elif type == 'err':
            rospy.logerr(full_message)


    def transitionToNextState(self):
        next_state = self.state + 1
        if next_state not in ALL_STATES:
            next_state = INACTIVE
        self.state = next_state

        if next_state == ENTERING_PARKING_LOT:
            self.log('ENTERING_PARKING_LOT')
            self.startNormalLaneFollowing()

        elif next_state == SEARCHING:
            self.log('SEARCHING')
            # Look on the left for Duckiebots backing out of parking spots
            self.toggleLEDDetection(led_detection_right=False)
            self.startNormalLaneFollowing()

        elif next_state == IS_PARKING:
            self.log('IS_PARKING')
            self.updateTopCutoff(80) # Cut out blue lines from other parking spots
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.updateDoffset(0.18) # Follow the left of the blue lane
            self.updateGain(0.5) # Go slower when entering parking spot
            self.setLEDs('blink') # Blink LEDs to indicate we are parking now
            self.pauseOperations(2) # Pause for 2 sec before continuing

        elif next_state == IS_PARKED:
            self.log('IS_PARKED')
            self.setLEDs('switchedoff') # Turn off LEDs while parked
            self.manualLaneControl('stop') # Stop lane control commands while parked

        elif next_state == EXITING_PARKING_SPOT:
            self.waitForRandomTime('exiting')
            self.log('EXITING_PARKING_SPOT')
            self.manualLaneControl('stop') # Force lane controller to stop
            self.setLEDs('red') # Set LEDs to indicate leaving parking
            self.pauseOperations(3) # Allow time for others to detect LEDs
            self.startBackwardsLaneFollowing() # Backwards wheel commands

        elif next_state == EXITING_PARKING_LOT:
            self.log('EXITING_PARKING_LOT')
            # Turn left to align with the lane
            self.manualLaneControl('left', duration=1.0)
            self.pauseOperations(2) # Pause for a few seconds
            # Look on the left for Duckiebots backing out of parking spots
            self.toggleLEDDetection(led_detection_right=False)
            self.startNormalLaneFollowing() # Resume normal lane following

    
    def waitForRandomTime(self, wait_type='searching'):
        """
        Divide the maneuver time in slots of 20 seconds
        so a duckiebot has to wait a random number of slots before performing
        the maneuver. The Duckiebot that is going around the parking area searching
        for parking, stops only if it knows that the one who want to exit can do the
        maneuver in the next 5 sec (time needed for the maneuver).
        """        
        secs = time.localtime().tm_sec
        slot = secs // self.time_slot_exiting
        delta = (slot + 1) * self.time_slot_exiting - secs
        rospy.log('Waiting a random time')
        if wait_type == 'exiting':
            wait = random.randrange(1, 4) * self.time_slot_exiting + delta
            self.log('Wait before exiting')
            self.pauseOperations(wait)
        else:
            slot_search = delta // self.time_slot_searching
            # stop only if it is in the first or in the last time slot,
            # because the car can go out
            if slot_search == 0 or slot_search == 3:
                self.pauseOperations(self.time_slot_searching)
                self.log('Wait before going straight')
            else:
                self.log('Can go straight')


    def pauseOperations(self, num_sec):
        # Pause lane control and parking for a few seconds
        self.log('Attempting to pause for %.1f seconds' % num_sec)
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def updateDoffset(self, new_offset):
        # Change the d_offset of the lane controller
        self.log('Publishing new d_offset: %.3f' % new_offset)
        self.d_offset_pub.publish(new_offset)


    def updateTopCutoff(self, cutoff=55):
        # Cutoff (don't examine) the top section of the image for line detector
        self.log('Setting line detector top_cutoff=%d' % cutoff)
        param_name = '/%s/line_detector_node/top_cutoff' % self.veh_name
        rospy.set_param(param_name, cutoff)


    def updateGain(self, gain):
        # Update the gain of the kinematics package
        self.log('Setting kinematic control gain=%.2f' % gain)
        param_name = '/%s/kinematics_node/gain' % self.veh_name
        rospy.set_param(param_name, gain)


    def updateLaneFilterColor(self, desired_color):
        # desired_color should be one of 'yellow', 'green', 'blue'
        self.log('Publishing new color for lane_filter: %s' % desired_color)
        self.lane_filter_color_pub.publish(desired_color)


    def toggleReversal(self, reverse):
        # Turn on/off backwards lane following
        self.log('Settings reversal to %s' % reverse)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = reverse
        self.reverse_pub.publish(msg)


    def toggleLEDDetection(self, led_detection_right):
        # Turn on/off LED detection
        self.log('Settings led_detection_right to %s' % led_detection_right)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = led_detection_right
        self.led_detection_right_pub.publish(msg)


    def pauseRedLineDetection(self, resume_after=2.0):
        # Pause the detection of red lines for a few seconds
        self.log('Pausing red line detection for %.1f seconds' % resume_after)
        def _cb(e):
            self.ignore_red_lines = False
        self.ignore_red_lines = True
        rospy.Timer(rospy.Duration.from_sec(resume_after), _cb, oneshot=True)


    def startBackwardsLaneFollowing(self):
        self.updateGain(1.0) # Go a bit faster when exiting parking spot
        self.updateTopCutoff(50) # Cut off upper image
        self.updateDoffset(0.118) # Follow center of lane
        self.updateLaneFilterColor('blue') # Follow blue lines
        self.toggleReversal(reverse=True) # Backwards wheel commands
        self.manualLaneControl('none') # No special turning maneuvers


    def startNormalLaneFollowing(self):
        self.toggleReversal(reverse=False) # No more reverse control
        self.setLEDs('switchedoff') # Set LEDs off while lane following
        self.updateDoffset(0) # d_offset=0 for normal lane following
        self.updateTopCutoff() # Default top cutoff for normal lane following
        self.updateLaneFilterColor('yellow') # Follow yellow lines (normal)
        self.manualLaneControl('none') # No special turning maneuvers
        self.updateGain(0.7) # Standardized gain


    def stopParkingModeFSM(self):
        # Transitions the FSM state out of parking mode
        self.log('Exiting FSM parking mode')
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.stop_parking_fsm_mode.publish(msg)


    def manualLaneControl(self, command, duration=None):
        if command not in ['straight', 'right', 'left', 'stop', 'none']:
            tup = (self.node_name, command)
            rospy.logerr('[%s] Invalid command: %s' % tup)
            return

        # Helper function to cancel any special commands
        def _deactivate():
            msg = String()
            msg.data = 'none'
            self.lane_control_override_pub.publish(msg)
            self.log('Finished command: %s' % command)

        if command == 'none':
            _deactivate()
            return

        self.log('Issuing command: %s' % command)
        msg = String()
        msg.data = command
        self.lane_control_override_pub.publish(msg)
        if duration is not None:
            # Deactivate the command after a certain duration
            rospy.sleep(duration)
            _deactivate()


    def setLEDs(self, pattern):
        if pattern not in ['white', 'red', 'switchedoff', 'blink']:
            self.log('Invalid LED pattern: %s' % pattern, type='err')
            return

        msg = LEDPattern()
        if pattern == 'blink':
            colors = ['white', 'red', 'white', 'red', 'white']
            color_mask = []
            frequency_mask = [1, 0, 1, 0, 1]
            frequency = 1.9
        else:
            colors = [pattern] * 5
            color_mask = [1] * 5
            frequency_mask = []
            frequency = 0

        msg.color_list = colors
        msg.color_mask = color_mask
        msg.frequency = frequency
        msg.frequency_mask = frequency_mask

        self.log('Settings LEDs to %s' % pattern)
        self.set_custom_led_pattern(msg)


if __name__ == '__main__':
    # create the node
    node = ParkingNode(node_name='parking_node')
    # keep spinning
    rospy.spin()
