#!/usr/bin/env python
from __future__ import print_function
import threading
import unittest

import rospy

from b2.msg import Proximity


PKG = 'b2'
NAME = 'base_node_test'

DEFAULT_PROXIMITY_TOPIC = "ir_sensors/proximity"
DEFAULT_LOOP_HZ = 10
DEFAULT_NUM_FLIPS = 4


class TestSensorsNode(unittest.TestCase):

    def __init__(self, *args):
        super(TestSensorsNode, self).__init__(*args)

        rospy.init_node("sensor_node_test", log_level=rospy.DEBUG, anonymous=True)
        self._loop_hz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)

        rospy.Subscriber(
            rospy.get_param("~proximity_topic", DEFAULT_PROXIMITY_TOPIC),
            Proximity,
            self._sensors_callback
        )

        self._prox_lock = threading.RLock()
        self._prox_msg = Proximity()
        self.looprate = rospy.Rate(self._loop_hz)

    def _sensors_callback(self, msg):
        with self._prox_lock:
            self._prox_msg = msg
            print("MSG: {}".format(self._prox_msg))

    def test_sensor_flips(self):
        # Watch sensor for a few seconds, and count the number of flips
        # The sensors should flip every 1 second

        numflips = rospy.get_param("~num_flips", DEFAULT_NUM_FLIPS)
        numloops = (numflips + 1) * self._loop_hz
        last_state = False
        flip_count = 0

        rospy.sleep(2)

        for i in range(numloops):
            with self._prox_lock:
                if self._prox_msg.sensors[0] != last_state:
                    flip_count += 1
                    last_state = self._prox_msg.sensors[0]
                    print("Flips counted: {}".format(flip_count))
            self.looprate.sleep()

        print("Actual: {}, Expected: {}".format(flip_count, numflips))
        self.assertGreaterEqual(
            flip_count, numflips,
        )


if __name__ == '__main__':

    import rostest
    rostest.rosrun(PKG, NAME, TestSensorsNode)
