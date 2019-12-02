#!/usr/bin/env python

import os
import rospy
import socket
import re
import pygame
import time
from duckietown import DTROS
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, WheelsCmdStamped
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
        self.pub_joystick = rospy.Publisher(
            '/%s/joy' % self.veh_name,
            Joy,
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
            '/%s/red_line' % self.veh_name,
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
        # We only care about stopping parking if we're currently parking
        if self.state != IS_PARKING:
            return

        should_stop_parking = msg.data == True
        #print(should_stop_parking)
        if should_stop_parking:
            rospy.loginfo('[%s] Stop parking maneuver!' % self.node_name)
            self.transitionToNextState()

            print("Parked, waiting for 5sec")
            self.pauseOperations(2)
            exitnow = False #self.waitForKeyboard()
            # print(exitnow)
            if exitnow == True:
                self.transitionToNextState()
            else:
                print("sth is fucked")



    def cbVehicleAvoidanceControl(self, twist):
        # We will only stop for another Duckiebot in certain states
        if self.state not in [SEARCHING]:
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
        if next_state not in ALL_STATES:
            next_state = INACTIVE

        if current_state == SEARCHING and next_state == IS_PARKING:
            self.updateLaneFilterColor('blue') # Follow blue lane, not yellow
            self.updateDoffset(0.18) # Follow the left of the lane
            self.blinkLEDs() # Blink LEDs to indicate we are parking now
            self.pauseOperations(2) # Pause for 2 sec before continuing
        elif current_state == IS_PARKING:
            print("within trans IS_PARKING")
            self.pauseOperations(2) # Stay in the parking spot 2 seconds
            # self.stopLaneFollowing()
            self.updateDoffset(0.11)
            self.pauseOperations(2)
            self.driveBackwards()
            print("after db")
            self.pauseOperations(2)
            self.updateDoffset(0.0)
            self.pauseOperations(2)
            self.updateLaneFilterColor('yellow')
            self.pauseOperations(2)
        elif current_state == IS_PARKED:
            self.pauseOperations(2)
            print("within trans IS_PARKED")
            # self.driveBackwards()
            # print("after drive bw")
            self.updateLaneFilterColor('yellow')
            # self.restartLaneFollowing()
            #self.waitForKeyboard()
        elif current_state == EXITING_PARKING_SPOT:
            pass
        else:
            pass # TODO - handle other states

        self.state = next_state


    def pauseOperations(self, num_sec):
        rospy.loginfo('[%s] Attempting to pause for %f seconds' % (self.node_name, num_sec))
        self.pause_lane_control_pub.publish(num_sec)
        rospy.sleep(num_sec)


    def driveBackwards(self):
        # car_control_msg = Twist2DStamped()
        # car_control_msg.v = -1.0
        # car_control_msg.omega = 0.0
        # self.pub_car_cmd.publish(car_control_msg)
        # msg_joy = Joy()
        # msg_joy.header.seq = 0
        # msg_joy.header.stamp.secs = 0
        # msg_joy.header.stamp.nsecs = 0
        # msg_joy.header.frame_id = ''
        # msg_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # msg_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # msg_joy.buttons[6] = 0
        # msg_joy.buttons[7] = 0
        # timeout = time.time() + 5   # 5 seconds from now
        # while True:
        #     msg_joy.axes[1] -= speed
        #     self.pub_joystick.publish(msg_joy)
        #     print("in while loop")
        #     if time.time() > timeout:
        #         break
        # print(msg_joy.axes[1])

        # self.stopLaneFollowing()
        # exitnow = False
        # while exitnow == False:
        #     exitnow = self.waitForKeyboard()
        #     print(exitnow)
        #
        # self.restartLaneFollowing()

        reverse = BoolStamped()
        reverse.header.stamp = rospy.Time.now()
        timeout = time.time() + 1.5   # 5 seconds from now
        while True:
            reverse.data = True
            self.pub_reverse.publish(reverse)
            # print(reverse.data)
            if time.time() > timeout:
                reverse.data = False
                self.pub_reverse.publish(reverse)
                print("time is over")
                break

        self.pauseOperations(2)
        return


    def waitForKeyboard(self):
        # return False
        try:

            input("Press enter to continue")
            return True
        except SyntaxError:
            return False


    def stopLaneFollowing(self):
        msg_joy = Joy()
        msg_joy.header.seq = 0
        msg_joy.header.stamp.secs = 0
        msg_joy.header.stamp.nsecs = 0
        msg_joy.header.frame_id = ''
        msg_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg_joy.axes[1] = 0
        msg_joy.buttons[6] = 1 #should send same msg as when pressing s

        self.pub_joystick.publish(msg_joy)
        print("lane following stopped")


    def restartLaneFollowing(self):
        msg_joy = Joy()
        msg_joy.header.seq = 0
        msg_joy.header.stamp.secs = 0
        msg_joy.header.stamp.nsecs = 0
        msg_joy.header.frame_id = ''
        msg_joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg_joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        msg_joy.axes[1] = 0
        msg_joy.buttons[6] = 0
        msg_joy.buttons[7] = 1 #should send same msg as when pressing a
        print("just before restart")
        self.pub_joystick.publish(msg_joy)
        print("just AFTER restart")


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
