#!/usr/bin/env python
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList, LanePose, BoolStamped, Twist2DStamped, FSMState
from duckietown_utils.instantiate_utils import instantiate
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
import json

class LaneFilterNode(object):

    def __init__(self):
        self.node_name = "Lane Filter"
        self.active = True
        self.filter = None
        self.matrix_delta_d = 0.2
        self.matrix_delta_phi = 0.1
        self.matrix_mesh_size = 1
        self.updateParams(None)

        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()

        self.d_median = []
        self.phi_median = []
        self.latencyArray = []

        # Since we have knowledge of maximum linear and angular velocity, we can have some idea
        # of when the enstimate are too far from what we may expect, and correct them
        # (basically applying a low pass filter)
        self.last_d = []
        self.last_phi = []
        self.veh_name = rospy.get_namespace().strip("/")
        self.omega_max = rospy.get_param("/" + self.veh_name + "/lane_controller_node/omega_max")
        self.omega_min = rospy.get_param("/" + self.veh_name +"/lane_controller_node/omega_min")
        self.v_ref = rospy.get_param("/" + self.veh_name + "/lane_controller_node/v_bar")
        self.gain = rospy.get_param("/" + self.veh_name + "/kinematics_node/gain")

        # Define Constants
        self.curvature_res = self.filter.curvature_res

        # Set parameters to server
        rospy.set_param('~curvature_res', self.curvature_res) #Write to parameter server for transparancy

        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)
        self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)
        self.sub_change_params = rospy.Subscriber("~change_params", String, self.cbChangeParams)
        # Publishers
        self.pub_lane_pose = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_seglist_filtered = rospy.Publisher("~seglist_filtered",SegmentList, queue_size=1)

        self.pub_ml_img = rospy.Publisher("~ml_img", Image, queue_size=1)


        self.pub_entropy = rospy.Publisher("~entropy",Float32, queue_size=1)


        # FSM
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
        self.active = True

        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)


    def cbChangeParams(self, msg):
        data = json.loads(msg.data)
        params = data["params"]
        reset_time = data["time"]
        # Set all paramters which need to be updated
        for param_name in params.keys():
            param_val = params[param_name]
            params[param_name] = eval("self.filter." + str(param_name))
            exec("self.filter." + str(param_name) + "=" + str(param_val))

        # Sleep for reset time
        rospy.sleep(reset_time)

        # Reset parameters to old values
        for param_name in params.keys():
            param_val = params[param_name]
            exec("self.filter." + str(param_name) + "=" + str(param_val))


    def updateParams(self, event):
        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new filter config: %s' % str(c))
            self.filter = instantiate(c[0], c[1])

        old_matrix_mesh_size = self.matrix_mesh_size

        #updating lane_offset
        self.lane_offset = rospy.get_param('~lane_offset', True)

        #updating matrix parameter delta_d and delta_phi
        self.matrix_mesh_size = rospy.get_param('~matrix_mesh_size', True)

        #changing the size of belief and masurement likelihood matrix
        if self.matrix_mesh_size != old_matrix_mesh_size:
            self.filter.update_matrix(self.matrix_mesh_size)
            self.loginfo('matrix_mesh_size %r' % self.matrix_mesh_size)


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data


    def processSegments(self,segment_list_msg):
        # Get actual timestamp for latency measurement
        timestamp_now = rospy.Time.now()


        if not self.active:
            return

        # TODO-TAL double call to param server ... --> see TODO in the readme, not only is it never updated, but it is alwas 0
        # Step 0: get values from server
        if (rospy.get_param('~curvature_res') is not self.curvature_res):
            self.curvature_res = rospy.get_param('~curvature_res')
            self.filter.updateRangeArray(self.curvature_res)

        # Step 1: predict
        current_time = rospy.get_time()
        dt = current_time - self.t_last_update
        v = self.velocity.v
        w = self.velocity.omega

        # ==== HERE THE UPPER BOUNDS ========
        ub_d = dt * self.omega_max * self.gain * 1000
        ub_phi = dt * self.v_ref * self.gain * 1000

        self.filter.predict(dt=dt, v=v, w=w)
        self.t_last_update = current_time

        # Step 2: update

        self.filter.update(segment_list_msg.segments, self.lane_offset)

        # Step 3: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimate()

        inlier_segments = self.filter.get_inlier_segments(segment_list_msg.segments, d_max, phi_max)
        inlier_segments_msg = SegmentList()
        inlier_segments_msg.header = segment_list_msg.header
        inlier_segments_msg.segments = inlier_segments
        self.pub_seglist_filtered.publish(inlier_segments_msg)


        max_val = self.filter.getMax()
        in_lane = max_val > self.filter.min_max
        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp

        lanePose.d = d_max[0]
        if not self.last_d:
            self.last_d = lanePose.d
        if lanePose.d - self.last_d > ub_d :
            lanePose.d = self.last_d + ub_d
            rospy.loginfo("d changed too much")
        if lanePose.d - self.last_d < -ub_d :
            lanePose.d = self.last_d - ub_d
            rospy.loginfo("d changed too much")

        lanePose.phi = phi_max[0]
        if not self.last_phi:
            self.last_phi = lanePose.phi
        if lanePose.phi - self.last_phi > ub_phi :
            lanePose.phi = self.last_phi + ub_phi
            rospy.loginfo("phi changed too much")
        if lanePose.phi - self.last_phi < -ub_phi :
            lanePose.phi = self.last_phi - ub_phi
            rospy.loginfo("phi changed too much")

        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL


        if self.curvature_res > 0:
            lanePose.curvature = self.filter.getCurvature(d_max[1:], phi_max[1:])

        self.pub_lane_pose.publish(lanePose)

        # Save for next iteration       <=========
        self.last_d = lanePose.d
        self.last_phi = lanePose.phi

        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg(np.array(255 * self.filter.beliefArray[0]).astype("uint8"), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp
        self.pub_belief_img.publish(belief_img)



        # Latency of Estimation including curvature estimation
        estimation_latency_stamp = rospy.Time.now() - timestamp_now
        estimation_latency = estimation_latency_stamp.secs + estimation_latency_stamp.nsecs/1e9
        self.latencyArray.append(estimation_latency)

        if (len(self.latencyArray) >= 20):
            self.latencyArray.pop(0)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = True #TODO-TAL change with in_lane. Is this messqge useful since it is alwas true ?
        self.pub_in_lane.publish(in_lane_msg)



    def cbMode(self, msg):
        return #TODO adjust self.active

    def updateVelocity(self,twist_msg):
        self.velocity = twist_msg

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")


    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter', anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
