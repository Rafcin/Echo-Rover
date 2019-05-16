#!/usr/bin/env python
import unittest
import threading

import rospy
from nav_msgs.msg import Odometry

from b2.msg import Proximity
from b2_logic.odometry_helpers import heading_from_odometry

PKG = 'b2'
NAME = 'pilot_node_test'

DEFAULT_SENSOR_TOPIC = "/ir_sensors/proximity"
DEFAULT_ODOM_TOPIC = "/base_node/odom"


class TestPilotNode(unittest.TestCase):
    def setUp(self):
        print()
        rospy.init_node(NAME, log_level=rospy.DEBUG, anonymous=True)
        self.state_lock = threading.RLock()
        self.odom = None  # type: Odometry

        rospy.Subscriber(
            rospy.get_param("~odom_topic", DEFAULT_ODOM_TOPIC),
            Odometry,
            self._odom_callback
        )

        self.pub_proximity = rospy.Publisher(
            rospy.get_param("~proximity_topic", DEFAULT_SENSOR_TOPIC),
            Proximity,
            queue_size=1
        )
        rospy.sleep(1)  # Let subscribers connect

    def test_fwd_obstacle(self):
        # Verify driving forward from (0, 0, 0)
        ok = False
        timeout_secs = 10
        for i in range(timeout_secs):
            print("   Secs: {}".format(i))
            heading, linear_x = self._get_heading_velocity()

            if -0.5 <= heading <= 0.5:
                if linear_x > 0.0:
                    ok = True
                    print("PASS) Forward heading and velocity OK")
                    break

            rospy.sleep(1)

        if not ok:
            self.fail(msg="Not driving forward at heading 0.0 after {} secs".format(timeout_secs))

        # Introduce obstacle
        self.pub_proximity.publish(Proximity(sensors=[True]))
        rospy.sleep(1)
        self.pub_proximity.publish(Proximity(sensors=[False]))

        # Verify now moving 90-deg to the right
        cnt = 0
        ok = False
        timeout_secs = 10
        for i in range(timeout_secs):
            print("   Secs: {}".format(i))
            heading, linear_x = self._get_heading_velocity()

            # Check that heading is around -1.571 for 3 secs
            if -1.600 <= heading <= -1.500:
                if linear_x > 0.0:
                    # We want to see this for 3 consecutive seconds
                    cnt = cnt + 1
                else:
                    cnt = 0  # Zero out if not consecutive
            else:
                cnt = 0  # Zero out if not consecutive

            if cnt >= 3:
                ok = True
                print("PASS) -90 deg heading and velocity OK")
                break
            rospy.sleep(1)
        if not ok:
            self.fail(msg="Not heading -90 deg for 3 consecutive seconds")

    def _odom_callback(self, msg):
        with self.state_lock:
            self.odom = msg

    def _get_heading_velocity(self):
        with self.state_lock:
            heading = heading_from_odometry(self.odom)
            linear_x = self.odom.twist.twist.linear.x
            print("   Heading: {}".format(heading))
            print("   X velocity: {}".format(linear_x))
        return heading, linear_x


if __name__ == '__main__':

    import rostest
    rostest.rosrun(PKG, NAME, TestPilotNode)
