
from collections import OrderedDict
from math import floor

from numpy.testing.utils import assert_almost_equal
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal, entropy

from std_msgs.msg import String
from duckietown_msgs.msg import SegmentList
import duckietown_utils as dtu

from duckietown_utils.parameters import Configurable

import numpy as np

from .lane_filter_interface import LaneFilterInterface

from .visualization import plot_phi_d_diagram_bgr

from scipy.stats import multivariate_normal
from scipy.ndimage.filters import gaussian_filter
from math import floor, sqrt
import copy

import rospy

# The `Segment` object in duckietown_msgs
# should be extended to include GREEN (3) and BLUE (4)
# but until then, we use these hard-coded values
YELLOW = 2
GREEN = 3
BLUE = 4

COLOR_MAPPING = {
    'yellow': YELLOW,
    'green': GREEN,
    'blue': BLUE
}
 

class LaneFilterHistogram(Configurable, LaneFilterInterface):
    #"""LaneFilterHistogram"""


    def __init__(self, configuration):
        param_names = [
            'mean_d_0',
            'mean_phi_0',
            'sigma_d_0',
            'sigma_phi_0',
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_max',
            'phi_min',
            'cov_v',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'min_max',
            'sigma_d_mask',
            'sigma_phi_mask',
            'curvature_res',
            'range_min',
            'range_est',
            'range_max',
            'curvature_right',
            'curvature_left',
        ]

        configuration = copy.deepcopy(configuration)
        Configurable.__init__(self, param_names, configuration)



        self.d, self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,
                                    self.phi_min:self.phi_max:self.delta_phi]
        self.d_pcolor, self.phi_pcolor = \
            np.mgrid[self.d_min:(self.d_max + self.delta_d):self.delta_d,
                     self.phi_min:(self.phi_max + self.delta_phi):self.delta_phi]


        self.beliefArray = []
        self.range_arr = np.zeros(self.curvature_res + 1)
        for i in range(self.curvature_res + 1):
            self.beliefArray.append(np.empty(self.d.shape))
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0 = [[self.sigma_d_0, 0], [0, self.sigma_phi_0]]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        self.d_med_arr = []
        self.phi_med_arr = []
        self.median_filter_size = 5

        self.default_delta_d = self.delta_d
        self.default_delta_phi = self.delta_phi

        self.update_matrix(1) #1 is initial value matrix_mash_size
        self.initialize()
        self.updateRangeArray(self.curvature_res)


        # Additional variables
        self.red_to_white = False
        self.is_dynamic = True
        self.range_est_min = 0
        self.filtered_segments = []

        # The dynamic color can be used to change which color
        # the Duckiebot observes during lane following, instead of yellow
        self.dynamic_color = YELLOW
        self.dynamic_color_sub = rospy.Subscriber(
            '/parking/lane_color',
            String,
            self.updateDynamicColor,
            queue_size=1
        )


    def updateDynamicColor(self, msg):
        old_color = self.dynamic_color
        dynamic_color_str = msg.data
        assert dynamic_color_str in COLOR_MAPPING.keys()
        self.dynamic_color = COLOR_MAPPING[dynamic_color_str]
        self.is_dynamic = True

        if old_color != self.dynamic_color:
            rospy.loginfo('LaneFilter color changed to %s' % dynamic_color_str)


    def update_matrix(self, param_mesh_size):

        self.delta_d = self.default_delta_d * param_mesh_size
        self.delta_phi = self.default_delta_phi * param_mesh_size
        self.d, self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,
                                    self.phi_min:self.phi_max:self.delta_phi]
        self.updateRangeArray(self.curvature_res)

    def getStatus(self):
        return LaneFilterInterface.GOOD

    def get_entropy(self):
        belief = self.beliefArray[0]
        s = entropy(belief.flatten())
        return s


    def predict(self, dt, v, w):
        delta_t = dt
        phi_t = self.phi + w * delta_t
        d_t = self.d + v * delta_t * np.sin(self.phi + 0.5*delta_t*w)   # Runge-Kutta


        for k in range(self.curvature_res+1):
            p_belief = np.zeros(self.beliefArray[k].shape)

            # there has got to be a better/cleaner way to do this - just applying the process model to translate each cell value
            for i in range(self.beliefArray[k].shape[0]):
                for j in range(self.beliefArray[k].shape[1]):
                    if self.beliefArray[k][i, j] > 0:
                        if d_t[i, j] > self.d_max or d_t[i, j] < self.d_min or phi_t[i, j] < self.phi_min or phi_t[i, j] > self.phi_max:
                            continue

                        i_new = int(
                            floor((d_t[i, j] - self.d_min) / self.delta_d))
                        j_new = int(
                            floor((phi_t[i, j] - self.phi_min) / self.delta_phi))

                        p_belief[i_new, j_new] += self.beliefArray[k][i, j]

            s_belief = np.zeros(self.beliefArray[k].shape)
            gaussian_filter(p_belief, self.cov_mask,
                            output=s_belief, mode='constant')

            if np.sum(s_belief) == 0:
                return
            self.beliefArray[k] = s_belief / np.sum(s_belief)


    def shouldIgnoreColor(self, color):
        # If the color is non-standard (i.e. not yellow)
        # we assume there are NOT white lines running parallel to the
        # colored line, and we ignore all colors except for the dynamic color

        if self.is_dynamic == False:
            return False

        if self.dynamic_color == GREEN and color != GREEN:
            return True
        elif self.dynamic_color == BLUE and color != BLUE:
            return True

        return False


    # prepare the segments for the creation of the belief arrays
    def prepareSegments(self, segments):
        segmentsRangeArray = map(list, [[]] * (self.curvature_res + 1))
        self.filtered_segments = []
        for segment in segments:
            if self.shouldIgnoreColor(segment.color):
                continue

            # Optional transform from RED to WHITE
            if self.red_to_white and segment.color == segment.RED:
                segment.color = segment.WHITE

            # Optional filtering out the dynamic color
            if not self.is_dynamic and segment.color == self.dynamic_color: continue

            # we don't care about RED ones for now
            if segment.color != segment.WHITE and segment.color != self.dynamic_color:
                continue
            # filter out any segments that are behind us
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue

            self.filtered_segments.append(segment)
            # only consider points in a certain range from the Duckiebot for the position estimation
            point_range = self.getSegmentDistance(segment)
            if point_range < self.range_est and point_range > self.range_est_min:
                segmentsRangeArray[0].append(segment)

            # split segments ot different domains for the curvature estimation
            if self.curvature_res is not 0:
                for i in range(self.curvature_res):
                    if point_range < self.range_arr[i + 1] and point_range > self.range_arr[i]:
                        segmentsRangeArray[i + 1].append(segment)
                        continue

        return segmentsRangeArray

    def updateRangeArray(self, curvature_res):
        self.curvature_res = curvature_res
        self.beliefArray = []
        for i in range(self.curvature_res + 1):
            self.beliefArray.append(np.empty(self.d.shape))
        self.initialize()
        if curvature_res > 1:
            self.range_arr = np.zeros(self.curvature_res + 1)
            range_diff = (self.range_max - self.range_min) / \
                (self.curvature_res)

            # populate range_array
            for i in range(len(self.range_arr)):
                self.range_arr[i] = self.range_min + (i * range_diff)

    # generate the belief arrays
    def update(self, segments, lane_offset):
        self.center_lane_offset = lane_offset
        # prepare the segments for each belief array
        segmentsRangeArray = self.prepareSegments(segments)
        # generate all belief arrays
        for i in range(self.curvature_res + 1):
            measurement_likelihood = self.generate_measurement_likelihood(segmentsRangeArray[i])

            if measurement_likelihood is not None:
                self.beliefArray[i] = np.multiply(self.beliefArray[i], measurement_likelihood)
                if np.sum(self.beliefArray[i]) == 0:
                    self.beliefArray[i] = measurement_likelihood
                else:
                    self.beliefArray[i] = self.beliefArray[i] / np.sum(self.beliefArray[i])



    def generate_measurement_likelihood(self, segments):

        # initialize measurement likelihood to all zeros
        measurement_likelihood = np.zeros(self.d.shape)
        filtered_segments = [s for s in segments if not self.shouldIgnoreColor(s.color)]
        for segment in filtered_segments:
            d_i, phi_i, l_i, weight =  self.generateVote(segment)

            # if the vote lands outside of the histogram discard it
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i > self.phi_max:
                continue

            i = int(floor((d_i - self.d_min) / self.delta_d))
            j = int(floor((phi_i - self.phi_min) / self.delta_phi))
            measurement_likelihood[i, j] = measurement_likelihood[i, j] + 1

        if np.linalg.norm(measurement_likelihood) == 0:
            return None
        measurement_likelihood = measurement_likelihood / \
            np.sum(measurement_likelihood)
        return measurement_likelihood

    # get the maximal values d_max and phi_max from the belief array. The first belief array (beliefArray[0]) includes the actual belief of the Duckiebots position. The further belief arrays are used for the curvature estimation.
    def getEstimate(self):
        d_max = np.zeros(self.curvature_res + 1)
        phi_max = np.zeros(self.curvature_res + 1)
        for i in range(self.curvature_res + 1):
            maxids = np.unravel_index(
                self.beliefArray[i].argmax(), self.beliefArray[i].shape)
            d_max[i] = self.d_min + (maxids[0] + 0.5) * self.delta_d
            phi_max[i] = self.phi_min + (maxids[1] + 0.5) * self.delta_phi
        return [d_max, phi_max]

    def get_estimate(self):
        d, phi = self.getEstimate()
        res = OrderedDict()
        res['d'] = d[0]
        res['phi'] = phi[0]
        return res

    # get the curvature estimation
    def getCurvature(self, d_max, phi_max):
        d_med_act = np.median(d_max) # actual median d value
        phi_med_act = np.median(phi_max) # actual median phi value

        # store median values over a few time step to smoothen out the estimation
        if len(self.d_med_arr) >= self.median_filter_size:
            self.d_med_arr.pop(0)
            self.phi_med_arr.pop(0)
        self.d_med_arr.append(d_med_act)
        self.phi_med_arr.append(phi_med_act)

        # set curvature
        # TODO: check magic constants
        if np.median(self.phi_med_arr) - phi_max[0] < -0.3 and np.median(self.d_med_arr) > 0.05:
            print "Curvature estimation: left curve"
            return self.curvature_left
        elif np.median(self.phi_med_arr) - phi_max[0] > 0.2 and np.median(self.d_med_arr) < -0.02:
            print "Curvature estimation: right curve"
            return self.curvature_right
        else:
            print "Curvature estimation: straight lane"
            return 0

    # return the maximal value of the beliefArray
    def getMax(self):
        return self.beliefArray[0].max()


    def initialize(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:, :, 0] = self.d
        pos[:, :, 1] = self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0, self.cov_0)
        for i in range(self.curvature_res + 1):
            self.beliefArray[i] = RV.pdf(pos)

    # generate a vote for one segment
    def generateVote(self, segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)

        n_hat = np.array([-t_hat[1], t_hat[0]])
        d1 = np.inner(n_hat, p1)
        d2 = np.inner(n_hat, p2)
        l1 = np.inner(t_hat, p1)
        l2 = np.inner(t_hat, p2)
        if (l1 < 0):
            l1 = -l1
        if (l2 < 0):
            l2 = -l2

        l_i = (l1 + l2) / 2
        d_i = (d1 + d2) / 2
        phi_i = np.arcsin(t_hat[1])

        if segment.color == segment.WHITE:  # right lane is white
            if(p1[0] > p2[0]):  # right edge of white lane
                d_i = d_i - self.linewidth_white
            else:  # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth

        elif segment.color == self.dynamic_color:
            if (p2[0] > p1[0]):  # left edge of lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else:  # right edge of white lane
                d_i = -d_i
            d_i = - d_i

        # weight = distance
        weight = 1

        #english driver parameter
        d_i += self.center_lane_offset

        return d_i, phi_i, l_i, weight

    def get_inlier_segments(self, segments, d_max, phi_max):
        inlier_segments = []
        filtered_segments = [s for s in segments if not self.shouldIgnoreColor(s.color)]
        for segment in filtered_segments:
            d_s, phi_s, l, w = self.generateVote(segment)
            if abs(d_s - d_max) < self.delta_d and abs(phi_s - phi_max)<self.delta_phi:
                inlier_segments.append(segment)
        return inlier_segments

    # get the distance from the center of the Duckiebot to the center point of a segment
    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x) / 2
        y_c = (segment.points[0].y + segment.points[1].y) / 2
        return sqrt(x_c**2 + y_c**2)

    def get_plot_phi_d(self, ground_truth=None):  # @UnusedVariable
        d, phi = self.getEstimate()
        belief = self.beliefArray[0]
        return plot_phi_d_diagram_bgr(self, belief, phi=phi, d=d)
