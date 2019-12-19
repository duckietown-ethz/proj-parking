#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import numpy
from duckietown_msgs.msg import FSMState, TagInfo, AprilTagsWithInfos, BoolStamped, TurnIDandType
from std_msgs.msg import String, Int16
import math

class RandomAprilTagTurnsNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.veh = os.environ['VEHICLE_NAME']
        self.turn_type = -1

        rospy.loginfo("[%s] Initializing." % (self.node_name))

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)
        self.pub_id_and_type = rospy.Publisher("~turn_id_and_type",TurnIDandType, queue_size=1, latch=True)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        #self.fsm_mode = None #TODO what is this?
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)

        # Subscribe to the FSM logic gate switch for parking, to control how the Duckiebot
        # will turn when it enters a parking intersection. If parking is on, it will turn
        # into the parking area, otherwise it will avoid turning into the parking area.
        self.parking_search_sub = rospy.Subscriber(
            "/%s/parking_on" % self.veh,
            BoolStamped,
            self.cbParkingSearch,
            queue_size=1
        )

        # Publish for each intersection, whether it is a parking intersection or a normal one.
        self.parking_intersection_pub = rospy.Publisher(
            "/%s/intersection_navigation_node/parking_intersection" % self.veh,
            BoolStamped,
            queue_size=1
        )

        self.searching_for_parking = False
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep", 1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)


        rospy.loginfo("[%s] Initialzed." % (self.node_name))

        self.rate = rospy.Rate(30)  # 10hz


    def cbParkingSearch(self, msg):
        # If this is true it means we are looking for the parking area
        # and so in the case of parking sign we take actions to enter
        self.searching_for_parking = msg.data


    def cbMode(self, mode_msg):
        #print mode_msg
        #TODO PUBLISH JUST ONCE
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            #rospy.loginfo("Turn type now: %i" %(self.turn_type))


    def cbTag(self, tag_msgs):
        valid_modes = [
            "INTERSECTION_CONTROL",
            "INTERSECTION_COORDINATION",
            "INTERSECTION_PLANNING"
        ]
        if self.fsm_mode in valid_modes:
            #loop through list of april tags

            # filter out the nearest apriltag
            dis_min = 999
            idx_min = -1
            for idx, taginfo in enumerate(tag_msgs.infos):
                if(taginfo.tag_type == taginfo.SIGN):
                    tag_det = (tag_msgs.detections)[idx]
                    pos = tag_det.pose.pose.position
                    distance = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                    if distance < dis_min:
                        dis_min = distance
                        idx_min = idx

            if idx_min != -1:
                taginfo = (tag_msgs.infos)[idx_min]

                availableTurns = []

                signType = taginfo.traffic_sign_type

                msg = BoolStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = False

                if self.searching_for_parking:

                    if signType == TagInfo.PARKING:
                        msg.data = True

                        if taginfo.id == 66:
                            availableTurns = [1] # Forward to enter parking area
                        elif taginfo.id == 63:
                            availableTurns = [0] # Left to enter parking area
                        elif taginfo.id == 10:
                            availableTurns = [2] # Right to enter parking area
                else:
                    #go through possible intersection types
                    if(signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT):
                        availableTurns = [0,1] # these mystical numbers correspond to the array ordering in open_loop_intersection_control_node (very bad)
                    elif (signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT):
                        availableTurns = [1,2]
                    elif (signType== taginfo.FOUR_WAY):
                        availableTurns = [0,1,2]
                    elif (signType == taginfo.T_INTERSECTION):
                        availableTurns = [0,2]

                        #now randomly choose a possible direction
                if(len(availableTurns)>0):
                    randomIndex = numpy.random.randint(len(availableTurns))
                    chosenTurn = availableTurns[randomIndex]
                    self.turn_type = chosenTurn
                    self.pub_turn_type.publish(self.turn_type)

                    id_and_type_msg = TurnIDandType()
                    id_and_type_msg.tag_id = taginfo.id
                    id_and_type_msg.turn_type = self.turn_type

                    # `True` if parking intersection, `False` otherwise
                    self.parking_intersection_pub.publish(msg)

                    self.pub_id_and_type.publish(id_and_type_msg)

                    #rospy.loginfo("possible turns %s." %(availableTurns))
                    #rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        #rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('random_april_tag_turns_node', anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
