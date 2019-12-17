#!/usr/bin/env python

import os
import rospy
import time
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
        self.state = ENTERING_PARKING_LOT
        self.at_red_line = False
        self.blob_detected = False
        self.ignore_red_lines = False

        # Services
        self.set_custom_led_pattern = rospy.ServiceProxy(
            '/%s/led_emitter_node/set_custom_pattern' % self.veh_name,
            SetCustomLEDPattern
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
        self.lane_control_override_pub = rospy.Publisher(
            '~/%s/parking/lane_control_override' % self.veh_name,
            String,
            queue_size=1
        )
        self.reverse_pub = rospy.Publisher(
            '~/%s/parking/reverse' % self.veh_name,
            BoolStamped,
            queue_size=1
        )
        self.led_detection_right_pub = rospy.Publisher(
            '~/%s/parking/led_detection_right' % self.veh_name,
            BoolStamped,
            queue_size=1
        )

        # Subscribers
        self.switch_sub = rospy.Subscriber(
            '~switch',
            BoolStamped,
            self.cbSwitch,
            queue_size=1
        )
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
        self.red_led_sub = rospy.Subscriber(
            '/%s/parking/red_led' % self.veh_name,
            BoolStamped,
            self.cbRedLED,
            queue_size=1
        )

        # start again without re-running
        self.restart_sub = rospy.Subscriber(
            '/%s/parking/start_from' % self.veh_name,
            BoolStamped,
            self.cbRestart,
            queue_size=1
        )

        self.log('Initialized.')

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """

    def cbRestart(self, msg):
        if msg.data:
            self.log('Resetting state to ENTERING_PARKING_LOT')
            self.state = INACTIVE
            self.at_red_line = False
            self.blob_detected = False
            self.ignore_red_lines = False
            self.transitionToNextState() # Transition to ENTERING_PARKING_LOT


    def cbSwitch(self, fsm_switch_msg):
        return

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
        # We only care about finding a spot if we're searching or exiting
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
                self.manualLaneControl('right', duration=0.5)
                # Resume lane following
                # TODO - Publish parking_off=True
                self.startNormalLaneFollowing()
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
                self.manualLaneControl('right', duration=0.5)
                # At intersection; automatically turn right to enter parking area
                self.manualLaneControl('none')
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
                self.manualLaneControl('straight', duration=2.0)
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
        self.pauseOperations(10) # Pause for some time till danger is gone
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
            self.pauseOperations(5) # Stay in the parking spot a certain time
            self.transitionToNextState() # Start exiting the parking spot

        elif next_state == EXITING_PARKING_SPOT:
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


    def pauseOperations(self, num_sec):
        self.log('Attempting to pause for %.1f seconds' % num_sec)
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def updateDoffset(self, new_offset):
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
        self.log('Settings reversal to %s' % reverse)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = reverse
        self.reverse_pub.publish(msg)


    def toggleLEDDetection(self, led_detection_right):
        self.log('Settings led_detection_right to %s' % led_detection_right)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = led_detection_right
        self.led_detection_right_pub.publish(msg)


    def pauseRedLineDetection(self, resume_after=2.0):
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
