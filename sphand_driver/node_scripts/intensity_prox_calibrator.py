#!/usr/bin/env python

import numpy as np

from sphand_driver.msg import IntensityProxCalibInfo
from sphand_driver.msg import IntensityProxCalibInfoArray
from sphand_driver.msg import ProximityStampedArray
from vl53l0x_mraa_ros.msg import RangingMeasurementDataStampedArray
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import rospy


class IntensityProxCalibrator(object):

    def __init__(self):
        self.i_prop_const = rospy.get_param('~i_prop_const', None)
        if self.i_prop_const is not None:
            self.i_prop_const = np.array(self.i_prop_const)
        self.i_init = rospy.get_param('~i_init', None)
        if self.i_init is not None:
            self.i_init = np.array(self.i_init)
        self.i_valid_min = rospy.get_param('~i_valid_min', 20)
        self.i_valid_max_dist = rospy.get_param('~i_valid_max_dist', 60)
        self.i_height_from_tof = rospy.get_param('~i_height_from_tof', None)
        if self.i_height_from_tof is not None:
            self.i_height_from_tof = np.array(self.i_height_from_tof)
        self.tof_valid_min = rospy.get_param('~tof_valid_min', 30)
        self.i_tof_tm_tolerance = rospy.get_param('~i_tof_tm_tolerance', 0.02)
        self.i_last_tms = None
        self.i_raw = None
        self.i_diff_from_init = None

        self.pub_i_calib = rospy.Publisher(
            '~output', IntensityProxCalibInfoArray,
            queue_size=1)
        self.sub_input_i = rospy.Subscriber(
            '~input/intensity', ProximityStampedArray, self._intensity_cb)
        self.sub_input_tof = rospy.Subscriber(
            '~input/tof', RangingMeasurementDataStampedArray, self._tof_cb)
        self.init_srv = rospy.Service(
            '~set_init_proximities', Trigger, self._set_init_proximities)

    def _intensity_cb(self, msg):
        self.i_last_tms = [p.header.stamp for p in msg.proximities]
        self.i_raw = np.array([p.proximity.proximity for p in msg.proximities])
        if self.i_init is None:
            rospy.logwarn_throttle(10, 'Init prox is not set, so skipping')
            return
        assert self.i_raw.shape == self.i_init.shape
        self.i_diff_from_init = self.i_raw - self.i_init
        if self.i_prop_const is None:
            rospy.logwarn_throttle(10, 'Prop const is not set, so skipping')
            return
        assert self.i_diff_from_init.shape == self.i_prop_const.shape
        diff_plus = self.i_diff_from_init.copy()
        diff_plus = diff_plus.astype(np.float64)
        diff_plus[diff_plus <= 0] = np.inf
        distance = np.sqrt(self.i_prop_const / diff_plus)
        distance[distance == 0] = np.inf

        # Publish calibrated info
        pub_msg = IntensityProxCalibInfoArray()
        pub_msg.header.stamp = msg.header.stamp
        for p, dist, diff, const, init in zip(msg.proximities,
                                              distance,
                                              self.i_diff_from_init,
                                              self.i_prop_const,
                                              self.i_init):
            info = IntensityProxCalibInfo()
            info.header.stamp = p.header.stamp
            info.distance = dist
            info.diff_from_init = diff
            info.prop_const = const
            info.init = init
            pub_msg.data.append(info)
        self.pub_i_calib.publish(pub_msg)

    def _tof_cb(self, msg):
        if self.i_diff_from_init is None:
            rospy.logwarn_throttle(
                10, 'Prox diff_from_init is not set, so skipping')
            return
        assert len(self.i_diff_from_init) == len(msg.array)
        assert len(msg.array) == len(self.i_last_tms)
        if self.i_height_from_tof is None:
            self.i_height_from_tof = [0.0] * len(msg.array)
        if self.i_prop_const is None:
            self.i_prop_const = np.zeros(len(msg.array))
        for i, data_st in enumerate(msg.array):
            if abs((data_st.header.stamp - self.i_last_tms[i]).to_sec()) > \
               self.i_tof_tm_tolerance:
                continue
            if self.i_diff_from_init[i] < self.i_valid_min:
                continue
            tof_r = data_st.data.range_millimeter
            if tof_r < self.tof_valid_min:
                continue
            if tof_r > self.i_valid_max_dist + self.i_height_from_tof[i]:
                continue
            self.i_prop_const[i] = self.i_diff_from_init[i] * (tof_r ** 2)

    def _set_init_proximities(self, req):
        is_success = True
        if self.i_raw is not None:
            self.i_init = self.i_raw.copy()
        else:
            is_success = False
        return TriggerResponse(success=is_success)


if __name__ == '__main__':
    rospy.init_node('intensity_prox_calibrator')
    app = IntensityProxCalibrator()
    rospy.spin()
