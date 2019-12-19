#!/usr/bin/env python

"""
As we encountered many problems with normal lane following
during the implementation of parking, and because the original
lane_controller_node.py is complex and somewhat unreadable,
most of this file has been rewritten. The major changes are as follows:

- Unused variables have been removed
- Random comments have been removed
- The code has been restructured to be more readable (same functionality)
- Backwards lane following was added, with different controller parameters
  when going backwards and the ability to switch backwards/forwards
- The controller state is reset when the FSM mode changes (found to be more stable)
- The controller's d_offset can be dynamically adjusted
- The controller can be paused (rospy.sleep)
- The controller can be forced to do hard-coded commands (e.g. right, left, straight)
"""

import os
import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import time
from std_msgs.msg import Float64, String
import numpy as np

class lane_controller(object):

    """
    #############################
    ####### INITIALIZATION ######
    #############################
    """

    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh = os.environ['VEHICLE_NAME']

        # Setup parameters
        self.velocity_to_m_per_s = 1.53
        self.omega_to_rad_per_s = 4.75
        self.resetControllerState()

        # Publications
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd",
            Twist2DStamped,
            queue_size=1
        )
        self.pub_actuator_limits_received = rospy.Publisher(
            "~actuator_limits_received",
            BoolStamped,
            queue_size=1
        )
        self.pub_radius_limit = rospy.Publisher(
            "~radius_limit",
            BoolStamped,
            queue_size=1
        )

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber(
            "~lane_pose",
            LanePose,
            self.cbPoseHandling,
            "lane_filter",
            queue_size=1
        )
        self.sub_intersection_navigation_pose = rospy.Subscriber(
            "~intersection_navigation_pose",
            LanePose,
            self.cbPoseHandling,
            "intersection_navigation",
            queue_size=1
        )
        self.sub_wheels_cmd_executed = rospy.Subscriber(
            "~wheels_cmd_executed",
            WheelsCmdStamped,
            self.cbUpdateWheelsCmdExecuted,
            queue_size=1
        )
        self.sub_actuator_limits = rospy.Subscriber(
            "~actuator_limits",
            Twist2DStamped,
            self.cbUpdateActuatorLimits,
            queue_size=1
        )
        self.sub_stop_line = rospy.Subscriber(
            "~stop_line_reading",
            StopLineReading,
            self.cbStopLineReading,
            queue_size=1
        )
        self.sub_doffset = rospy.Subscriber(
            "~doffset",
            Float64,
            self.cbDoffset,
            queue_size=1
        )
        self.sub_pause_ops = rospy.Subscriber(
            "~pause",
            Float64,
            self.cbPauseOperations,
            queue_size=1
        )
        self.sub_reverse = rospy.Subscriber(
            '/%s/parking/reverse' % self.veh,
            BoolStamped,
            self.cbReverse,
            queue_size=1
        )
        self.sub_manual_override = rospy.Subscriber(
            '/%s/parking/lane_control_override' % self.veh,
            String,
            self.cbManualOverride,
            queue_size=1
        )

        # FSM
        self.sub_switch = rospy.Subscriber(
            "~switch",
            BoolStamped,
            self.cbSwitch,
            queue_size=1
        )
        self.sub_fsm_mode = rospy.Subscriber(
            "~fsm_mode",
            FSMState,
            self.cbMode,
            queue_size=1
        )

        self.msg_radius_limit = BoolStamped()
        self.msg_radius_limit.data = self.use_radius_limit
        self.pub_radius_limit.publish(self.msg_radius_limit)

        # safe shutdown
        rospy.on_shutdown(self.custom_shutdown)

        self.gains_timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.getGains_event)
        rospy.loginfo("[%s] Initialized " % (rospy.get_name()))


    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.logdebug("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


    def resetControllerState(self):
        self.lane_reading = None
        self.last_ms = None
        self.pub_counter = 0
        self.should_reverse = False
        self.stop_line_distance = 999
        self.stop_line_detected = False
        self.past_time = 0
        self.manual_command = None
        self.setGains()


    def setGains(self):
        self.v_bar_gain_ref = 0.5
        self.v_max = 1

        v_bar_fallback = 0.25  # nominal speed, 0.25m/s
        k_theta_fallback = -2.0
        k_d_fallback = - (k_theta_fallback ** 2) / (4.0 * self.v_bar_gain_ref)

        theta_thres_fallback = math.pi / 6.0
        d_thres_fallback = math.fabs(k_theta_fallback / k_d_fallback) * theta_thres_fallback
        d_offset_fallback = 0.0

        k_Id_fallback = 2.5
        k_Iphi_fallback = 1.25

        self.fsm_state = None
        self.cross_track_err = 0
        self.heading_err = 0
        self.cross_track_integral = 0
        self.heading_integral = 0
        self.cross_track_integral_top_cutoff = 0.3
        self.cross_track_integral_bottom_cutoff = -0.3
        self.heading_integral_top_cutoff = 1.2
        self.heading_integral_bottom_cutoff = -1.2
        self.time_start_curve = 0

        self.wheels_cmd_executed = WheelsCmdStamped()

        self.actuator_limits = Twist2DStamped()
        self.actuator_limits.v = 999.0  # to make sure the limit is not hit before the message is received
        self.actuator_limits.omega = 999.0  # to make sure the limit is not hit before the message is received
        self.omega_max = 999.0  # considering radius limitation and actuator limits

        self.use_radius_limit_fallback = True

        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.v_ref_possible = {'default': self.v_max, 'main_pose': v_bar_fallback}
        self.main_pose_source = None

        self.active = True

        # overwrites some of the above set default values (the ones that are already defined in the corresponding yaml-file (see launch-file of this node))

        # Linear velocity
        self.v_bar = self.setupParameter("~v_bar", v_bar_fallback)
        # P gain for d
        self.k_d = self.setupParameter("~k_d", k_d_fallback)
        # P gain for theta
        self.k_theta = self.setupParameter("~k_theta", k_theta_fallback)
        # Cap for error in d
        self.d_thres = self.setupParameter("~d_thres", d_thres_fallback)
        # Maximum desired theta
        self.theta_thres = self.setupParameter("~theta_thres", theta_thres_fallback)
        # A configurable offset from the lane position
        self.d_offset = self.setupParameter("~d_offset", d_offset_fallback)

        # Gain for integrator of d
        self.k_Id = self.setupParameter("~k_Id", k_Id_fallback)
        # Gain for integrator of phi (phi = theta)
        self.k_Iphi = self.setupParameter("~k_Iphi",k_Iphi_fallback)

        # setup backward parameters
        self.k_d_back = self.setupParameter("~k_d_back",3.0)
        self.k_theta_back = self.setupParameter("~k_theta_back",1.0)
        self.k_Id_back = self.setupParameter("~k_Id_back",1.0)
        self.k_Itheta_back = self.setupParameter("~k_Itheta_back",-1.0)


        self.omega_ff = self.setupParameter("~omega_ff", 0)
        self.omega_max = self.setupParameter("~omega_max", 4.7)
        self.omega_min = self.setupParameter("~omega_min", -4.7)
        self.use_radius_limit = self.setupParameter("~use_radius_limit", self.use_radius_limit_fallback)
        self.min_radius = self.setupParameter("~min_rad", 0.0)

        self.d_ref = self.setupParameter("~d_ref", 0)
        self.phi_ref = self.setupParameter("~phi_ref",0)
        self.object_detected = self.setupParameter("~object_detected", 0)


    def getGains_event(self, event):
        v_bar = rospy.get_param("~v_bar")
        k_d = rospy.get_param("~k_d")
        k_theta = rospy.get_param("~k_theta")
        d_thres = rospy.get_param("~d_thres")
        theta_thres = rospy.get_param("~theta_thres")
        d_offset = rospy.get_param("~d_offset")
        d_ref = rospy.get_param("~d_ref")
        phi_ref = rospy.get_param("~phi_ref")
        use_radius_limit = rospy.get_param("~use_radius_limit")
        object_detected = rospy.get_param("~object_detected")

        # get backward parameters
        k_d_back = rospy.get_param("~k_d_back")
        k_theta_back = rospy.get_param("~k_theta_back")
        k_Id_back = rospy.get_param("~k_Id_back")
        k_Itheta_back = rospy.get_param("~k_Itheta_back")

        self.omega_ff = rospy.get_param("~omega_ff")
        self.omega_max = rospy.get_param("~omega_max")
        self.omega_min = rospy.get_param("~omega_min")
        self.velocity_to_m_per_s = 1

        # FeedForward
        self.curvature_outer = 1 / (0.39)
        self.curvature_inner = 1 / 0.175

        k_Id = rospy.get_param("~k_Id")
        k_Iphi = rospy.get_param("~k_Iphi")

        if self.k_Id != k_Id:
            rospy.loginfo("ADJUSTED I GAIN")
            self.cross_track_integral = 0
            self.k_Id = k_Id

        params_old = (self.v_bar, self.k_d, self.k_theta, self.d_thres, self.theta_thres,
            self.d_offset, self.k_Id, self.k_Iphi, self.use_radius_limit,
            self.k_d_back,self.k_theta_back,self.k_Id_back,self.k_Itheta_back)
        params_new = (v_bar, k_d, k_theta, d_thres, theta_thres, d_offset,
            k_Id, k_Iphi, use_radius_limit,
            k_d_back,k_theta_back,k_Id_back,k_Itheta_back)

        if params_old != params_new:
            rospy.loginfo("[%s] Gains changed." % (self.node_name))

            self.d_ref = d_ref
            self.phi_ref = phi_ref
            self.theta_thres = theta_thres
            self.d_offset = d_offset
            self.v_bar = v_bar
            self.d_thres = d_thres

            self.k_d = k_d
            self.k_theta = k_theta

            self.k_Id = k_Id
            self.k_Iphi = k_Iphi

            # setting backward parameters
            self.k_d_back = k_d_back
            self.k_theta_back = k_theta_back

            self.k_Id_back = k_Id_back
            self.k_Itheta_back = k_Itheta_back

            if use_radius_limit != self.use_radius_limit:
                self.use_radius_limit = use_radius_limit
                self.msg_radius_limit.data = self.use_radius_limit
                self.pub_radius_limit.publish(self.msg_radius_limit)

    """
    #############################
    ######### CALLBACKS #########
    #############################
    """

    def cbMode(self,fsm_state_msg):
        self.fsm_state = fsm_state_msg.state # String of current FSM state
        rospy.loginfo('fsm_state changed in lane_controller_node to: %s' % self.fsm_state)
        self.resetControllerState()
        rospy.loginfo('[%s] Controller state has been reset.' % self.node_name)


    def cbDoffset(self, msg):
        rospy.set_param("~d_offset", msg.data)
        self.d_offset = msg.data
        rospy.loginfo('[%s] Updating d_offset to %f' % (self.node_name, self.d_offset))


    def cbPauseOperations(self, msg):
        num_sec = msg.data
        rospy.loginfo('[%s] Pausing operations for %.1f seconds' % (self.node_name, num_sec))
        self.active = False
        self.sendStop()
        rospy.sleep(num_sec)
        self.active = True


    def cbStopLineReading(self, msg):
        self.stop_line_distance = np.sqrt(
            msg.stop_line_point.x**2 + msg.stop_line_point.y**2 + msg.stop_line_point.z**2
        )
        self.stop_line_detected = msg.stop_line_detected


    def cbManualOverride(self, msg):
        command = msg.data.lower()
        if command not in ['straight', 'right', 'left', 'stop', 'none']:
            return
        if command == 'none':
            self.manual_command = None
        else:
            self.manual_command = command
        tup = (self.node_name, str(self.manual_command))
        rospy.loginfo('[%s] Set manual command to %s' % tup)


    def cbReverse(self, msg):
        should_reverse = msg.data
        tup = (self.node_name, should_reverse)
        rospy.loginfo('[%s] Reverse = %s' % tup)
        self.should_reverse = should_reverse


    def cbSwitch(self, fsm_switch_msg):
        self.active = fsm_switch_msg.data # True or False
        rospy.loginfo("active: " + str(self.active))


    def cbPoseHandling(self, input_pose_msg, pose_source):
        if not self.active:
            return

        self.prev_pose_msg = self.pose_msg
        self.pose_msg_dict[pose_source] = input_pose_msg

        if self.pose_initialized:
            v_ref_possible_default = self.v_ref_possible["default"]
            v_ref_possible_main_pose = self.v_ref_possible["main_pose"]
            self.v_ref_possible.clear()
            self.v_ref_possible["default"] = v_ref_possible_default
            self.v_ref_possible["main_pose"] = v_ref_possible_main_pose

        if self.fsm_state == "INTERSECTION_CONTROL" and pose_source == "intersection_navigation":
            self.pose_msg = input_pose_msg
            self.pose_msg.curvature_ref = input_pose_msg.curvature
            self.v_ref_possible["main_pose"] = self.v_bar
            self.main_pose_source = pose_source
            self.pose_initialized = True
        elif pose_source == "lane_filter":
            #rospy.loginfo("pose source: lane_filter")
            self.pose_msg = input_pose_msg
            self.pose_msg.curvature_ref = input_pose_msg.curvature

            self.v_ref_possible["main_pose"] = self.v_bar

            # Adapt speed to stop line!
            if self.stop_line_detected:
                # 60cm -> v_bar, 15cm -> v_bar/2
                d1, d2 = 0.8, 0.25
                a = self.v_bar/(2*(d1-d2))
                b = self.v_bar - a*d1
                v_new = a*self.stop_line_distance + b
                v_new = np.max([self.v_bar/2.0, np.min([self.v_bar, v_new])])
                self.v_ref_possible["main_pose"] = v_new

            self.main_pose_source = pose_source
            self.pose_initialized = True

        self.pose_msg.v_ref = min(self.v_ref_possible.itervalues())

        if self.pose_msg != self.prev_pose_msg and self.pose_initialized:
            self.updatePose(self.pose_msg)


    def cbUpdateWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd


    def cbUpdateActuatorLimits(self, msg_actuator_limits):
        self.actuator_limits = msg_actuator_limits
        rospy.logdebug("actuator limits updated to: ")
        rospy.logdebug("actuator_limits.v: " + str(self.actuator_limits.v))
        rospy.logdebug("actuator_limits.omega: " + str(self.actuator_limits.omega))
        msg_actuator_limits_received = BoolStamped()
        msg_actuator_limits_received.data = True
        self.pub_actuator_limits_received.publish(msg_actuator_limits_received)

    """
    #############################
    ###### HELPER FUNCTIONS #####
    #############################
    """

    def sendStop(self):
        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)


    def custom_shutdown(self):
        rospy.loginfo("[%s] Shutting down..." % self.node_name)

        # Stop listening
        self.sub_lane_reading.unregister()
        self.sub_intersection_navigation_pose.unregister()
        self.sub_wheels_cmd_executed.unregister()
        self.sub_actuator_limits.unregister()
        self.sub_switch.unregister()
        self.sub_fsm_mode.unregister()

        # Send stop command
        self.sendStop()

        rospy.sleep(0.5) # To make sure that it gets published.
        rospy.loginfo("[%s] Shutdown" %self.node_name)


    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


    def updatePose(self, pose_msg):
        if self.should_reverse:
            backward = -1
            k_d=self.k_d_back #= 3
            k_theta=self.k_theta_back #= 1
            k_Id=self.k_Id_back #= -1
            k_Iphi=self.k_Itheta_back #= -1
        else:
            backward = 1
            k_d=self.k_d #= -3
            k_theta=self.k_theta #= -1
            k_Id=self.k_Id #= 1
            k_Iphi=self.k_Iphi #= 0


        self.lane_reading = pose_msg

        # Calculating the delay image processing took
        timestamp_now = rospy.Time.now()
        image_delay_stamp = timestamp_now - self.lane_reading.header.stamp

        # delay from taking the image until now in seconds
        image_delay = image_delay_stamp.secs + image_delay_stamp.nsecs / 1e9

        prev_cross_track_err = self.cross_track_err
        prev_heading_err = self.heading_err

        self.cross_track_err = pose_msg.d - self.d_offset
        self.heading_err = backward * pose_msg.phi

        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        car_control_msg.v = pose_msg.v_ref

        if car_control_msg.v > self.actuator_limits.v:
            car_control_msg.v = self.actuator_limits.v

        if math.fabs(self.cross_track_err) > self.d_thres:
            rospy.logerr("Inside threshold")
            self.cross_track_err = np.sign(self.cross_track_err) * self.d_thres

        currentMillis = int(round(time.time() * 1000))

        if self.last_ms is not None:
            dt = (currentMillis - self.last_ms) / 1000.0
            self.cross_track_integral += self.cross_track_err * dt
            self.heading_integral += self.heading_err * dt

        if self.cross_track_integral > self.cross_track_integral_top_cutoff:
            self.cross_track_integral = self.cross_track_integral_top_cutoff
        if self.cross_track_integral < self.cross_track_integral_bottom_cutoff:
            self.cross_track_integral = self.cross_track_integral_bottom_cutoff

        if self.heading_integral > self.heading_integral_top_cutoff:
            self.heading_integral = self.heading_integral_top_cutoff
        if self.heading_integral < self.heading_integral_bottom_cutoff:
            self.heading_integral = self.heading_integral_bottom_cutoff

        if abs(self.cross_track_err) <= 0.011:  # TODO: replace '<= 0.011' by '< delta_d' (but delta_d might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.cross_track_integral = 0
        if abs(self.heading_err) <= 0.051:  # TODO: replace '<= 0.051' by '< delta_phi' (but delta_phi might need to be sent by the lane_filter_node.py or even lane_filter.py)
            self.heading_integral = 0
        if np.sign(self.cross_track_err) != np.sign(prev_cross_track_err):  # sign of error changed => error passed zero
            self.cross_track_integral = 0
        if np.sign(self.heading_err) != np.sign(prev_heading_err):  # sign of error changed => error passed zero
            self.heading_integral = 0
        if self.wheels_cmd_executed.vel_right == 0 and self.wheels_cmd_executed.vel_left == 0:  # if actual velocity sent to the motors is zero
            self.cross_track_integral = 0
            self.heading_integral = 0

        omega_feedforward = car_control_msg.v * pose_msg.curvature_ref
        if self.main_pose_source == "lane_filter":
            omega_feedforward = 0

        # Scale the parameters linear such that their real value is at 0.22m/s
        omega = k_d * (0.22/self.v_bar) * self.cross_track_err + k_theta * (0.22/self.v_bar) * self.heading_err
        omega += backward * omega_feedforward

        # check if nominal omega satisfies min radius, otherwise constrain it to minimal radius
        if math.fabs(omega) > car_control_msg.v / self.min_radius:
            if self.last_ms is not None:
                self.cross_track_integral -= self.cross_track_err * dt
                self.heading_integral -= self.heading_err * dt
            omega = math.copysign(car_control_msg.v / self.min_radius, omega)

        if not self.fsm_state == "SAFE_JOYSTICK_CONTROL":
            # apply integral correction (these should not affect radius, hence checked afterwards)
            omega -= k_Id * (0.22/self.v_bar) * self.cross_track_integral
            omega -= k_Iphi * (0.22/self.v_bar) * self.heading_integral

        if car_control_msg.v == 0:
            omega = 0
        elif car_control_msg.v - 0.5 * math.fabs(omega) * 0.1 < 0.065:
            # check if velocity is large enough such that car can actually execute desired omega
            car_control_msg.v = 0.065 + 0.5 * math.fabs(omega) * 0.1

        # apply magic conversion factors
        car_control_msg.v = backward * car_control_msg.v * self.velocity_to_m_per_s
        omega = omega * self.omega_to_rad_per_s

        if omega > self.omega_max: omega = self.omega_max
        if omega < self.omega_min: omega = self.omega_min
        omega += backward * self.omega_ff
        car_control_msg.omega = omega

        if self.manual_command == 'stop':
            car_control_msg.omega = 0.0
            car_control_msg.v = 0.0
        elif self.manual_command == 'straight':
            car_control_msg.omega = 0.0
            car_control_msg.v = 0.23
        elif self.manual_command == 'right':
            car_control_msg.omega = -3.7
            car_control_msg.v = 0.28
        elif self.manual_command == 'left':
            car_control_msg.omega = -3
            car_control_msg.v = -0.15

        self.publishCmd(car_control_msg)
        self.last_ms = currentMillis


if __name__ == "__main__":
    rospy.init_node("lane_controller_node", anonymous=False)
    lane_control_node = lane_controller()
    rospy.spin()
