import math
import unittest

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from b2_logic.nodes.base import BaseNode
from b2_logic.odometry_helpers import heading_from_odometry
from roboclaw_driver.msg import Stats


PKG = 'b2'
NAME = 'b2_base_unittest'


class MockSpeedCmdPub():
    # Motor speed in QPPS
        # m1_qpps
        # m2_qpps

    # Max seconds before automatically stopping
        # max_secs
    def __init__(self):
        self.msg = None

    def publish(self, msg):
        self.msg = msg
        print("<- SpeedCommand: m1_qpps: {}, m2_qpps: {}, max_secs: {}".format(
            msg.m1_qpps, msg.m2_qpps, msg.max_secs))


class MockOdomPub():
    def __init__(self):
        self.msg = None

    def publish(self, msg):
        self.msg = msg


class MockTfBroadcaster():
    def sendTransform(
        self, (world_x, world_y, world_z), (quat_x, quat_y, quat_z, quat_w),
        nowtime, base_frame_id, world_frame_id
    ):
        pass


class TestBaseLoop(unittest.TestCase):
    def setUp(self):
        print()
        wheel_dist = 0.220
        wheel_radius = 65 / 1000.0 / 2.0  # 65mm dia wheels
        ticks_per_rotation = 48 * 34
        max_drive_secs = 1
        deadman_secs = 1
        max_qpps = 3700
        base_frame_id = "base_link"
        world_frame_id = "world"

        self.speed_cmd_pub = MockSpeedCmdPub()
        self.odom_pub = MockOdomPub()
        self.tf_broadcaster = MockTfBroadcaster()

        self.base = BaseNode(
            wheel_dist, wheel_radius, ticks_per_rotation,
            max_drive_secs, deadman_secs, max_qpps,
            base_frame_id, world_frame_id,
            self.speed_cmd_pub, self.odom_pub, self.tf_broadcaster
        )

        self._rostime = rospy.Time(0)
        self._monkey_patch_rospy_get_time()

        # Starting state
        self.base._m1_enc_prev = 0
        self.base._m2_enc_prev = 0
        self.base._last_odom_time = rospy.get_rostime()

    def test_start_zero(self):
        self.base._last_cmd_vel_time = None
        self.base.process_base_loop()
        self._compare_speed_command(self.speed_cmd_pub.msg, 0, 0)
        self._compare_odometry(self.odom_pub.msg, 0, 0, 0, 0, 0, 0)

    def test_move_forward(self):

        # Send drive fwd at 1.0 m/s
        self._send_cmd_vel(1.0, 0.0)
        self._set_rostime(1)
        self._send_stats(0, 0)
        self.base._last_cmd_vel_time = rospy.get_rostime()
        self.base.process_base_loop()
        self._compare_speed_command(self.speed_cmd_pub.msg, 3700, 3700)
        self._compare_odometry(self.odom_pub.msg, 0, 0, 0, 0, 0, 0)

        # Drive one wheel roll forward in 1 second
        # starting from origin (0,0) and 0 heading
        # Expect odom distance-x to be one wheel role
        # Expect odom velocity-x to be 2 wheel roles per sec
        self._add_rostime(0.5)
        self._send_stats(1632, 1632)
        self.base.process_base_loop()
        wheel_roll_dist = (0.065 / 2) * (2 * math.pi)  # 2pi * R
        self._compare_odometry(self.odom_pub.msg, wheel_roll_dist, 0, 0, wheel_roll_dist * 2, 0, 0)

    #
    # Helper methods
    #
    def _compare_odometry(self, odom, exp_world_x, exp_world_y, exp_world_theta,
                          exp_world_linear_x, exp_world_linear_y, exp_world_angular_z):
        if odom is None:
            print("ERROR: Odometry is None, skipping compare")
            return

        print("Odom:")
        pose = odom.pose.pose
        world_x = pose.position.x
        world_y = pose.position.y
        world_theta = heading_from_odometry(odom)
        print("   World position: ({}, {}, {}), expected ({}, {}, {})".format(
            round(world_x, 3), round(world_y, 3), round(world_theta, 3),
            round(exp_world_x, 3), round(exp_world_y, 3), round(exp_world_theta, 3),
        ))
        self.assertAlmostEqual(world_x, exp_world_x, places=0)
        self.assertAlmostEqual(world_y, exp_world_y, places=0)
        self.assertAlmostEqual(world_theta, exp_world_theta, places=0)

        twist = odom.twist.twist
        world_linear_x = twist.linear.x
        world_linear_y = twist.linear.y
        world_angular_z = twist.angular.z
        print("   World velocity: ({}, {}, {}), expected ({}, {}, {})".format(
            round(world_linear_x, 3), round(world_linear_y, 3), round(world_angular_z, 3),
            round(exp_world_linear_x, 3), round(exp_world_linear_y, 3),
            round(exp_world_angular_z, 3),
        ))
        self.assertAlmostEqual(world_linear_x, exp_world_linear_x, places=3)
        self.assertAlmostEqual(world_linear_y, exp_world_linear_y, places=3)
        self.assertAlmostEqual(world_angular_z, exp_world_angular_z, places=3)

    def _compare_speed_command(self, speed_cmd_msg, exp_m1_qpps, exp_m2_qpps):
        print("Stats:")
        print("   M1 QPPS actual: {}, expected: {}".format(
            speed_cmd_msg.m1_qpps, exp_m1_qpps))
        print("   M2 QPPS actual: {}, expected: {}".format(
            speed_cmd_msg.m2_qpps, exp_m2_qpps))
        self.assertAlmostEqual(speed_cmd_msg.m1_qpps, exp_m1_qpps, delta=10)
        self.assertAlmostEqual(speed_cmd_msg.m2_qpps, exp_m2_qpps, delta=10)

    def _send_stats(self, m1_enc_val, m2_enc_val):
        stats = Stats()
        stats.m1_enc_val = m1_enc_val
        stats.m2_enc_val = m2_enc_val
        stats.header.stamp = rospy.get_rostime()
        self.base.roboclaw_stats_callback(stats)
        stats2 = self.base._roboclaw_stats
        print("-> Stats: m1_enc_val: {}, m2_enc_val: {}, m1_enc_qpps: {}, "
              "m2_enc_qpps: {}, time {}".format(
                stats2.m1_enc_val, stats2.m2_enc_val,
                stats2.m1_enc_qpps, stats2.m2_enc_qpps, stats2.header.stamp.to_sec()))

    def _send_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.base.cmd_vel_callback(twist)
        print("-> Twist: linear.x: {}, angular.z: {}".format(
            self.base._x_linear_cmd, self.base._z_angular_cmd))

    def _monkey_patch_rospy_get_time(self):
        def x():
            return self._rostime
        rospy.get_rostime = x

    def _set_rostime(self, secs):
        self._rostime = rospy.Time(secs)
        print("ROS time: {} secs".format(self._rostime.to_sec()))

    def _add_rostime(self, secs):
        self._set_rostime((self._rostime.to_sec() + secs))


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestBaseLoop)
