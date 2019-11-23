#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64
from duckietown_msgs.msg import BoolStamped


class ParkingNode(DTROS):

	def __init__(self, node_name):
		# initialize the DTROS parent class
		super(ParkingNode, self).__init__(node_name=node_name)

		self.veh_name = rospy.get_namespace().strip("/")
		self.node_name = 'ParkingNode'

		offset_mode = 2
		if offset_mode == 0:
			self.offset = 0.0
		elif offset_mode == 1:
			self.offset = 0.2175	# lane distance
		elif offset_mode == 2:
			self.offset = 0.115	 # middle of the road
		else:
			self.offset = 0.0

		# Publishers
		self.pub = rospy.Publisher('lane_controller_node/doffset', Float64, queue_size=1)

		# Subscribers
		self.sub = rospy.Subscriber(
			'/%s/vehicle_detection_node/detection' % self.veh_name,
			BoolStamped,
			self.cbOvertake,
			queue_size=1
		)

		rospy.loginfo('%s initialized' % self.node_name)


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
