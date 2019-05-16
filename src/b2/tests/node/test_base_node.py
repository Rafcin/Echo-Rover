#!/usr/bin/env python
from __future__ import print_function
from datetime import datetime as dt
import unittest
import math

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations

from b2_logic.odometry_helpers import yaw_from_odom_message

PKG = 'b2'
NAME = 'base_node_test'

DEFAULT_TWIST_TOPIC = "/base_node/cmd_vel"
DEFAULT_ODOM_TOPIC = "/base_node/odom"
DEFAULT_LOOP_HZ = 10


class TestBaseNode(unittest.TestCase):

    def __init__(self, *args):
        super(TestBaseNode, self).__init__(*args)

        rospy.init_node(NAME, log_level=rospy.DEBUG, anonymous=True)
        self._loop_hz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)

        self.odom = Odometry()

        rospy.Subscriber(
            rospy.get_param("~odom_topic", DEFAULT_ODOM_TOPIC),
            Odometry,
            self._odom_callback
        )

        self.pub_cmd_vel = rospy.Publisher(
            rospy.get_param("~cmd_vel_topic", DEFAULT_TWIST_TOPIC),
            Twist,
            queue_size=1
        )
        rospy.sleep(1)  # Let subscribers connect

    def _odom_callback(self, msg):
        self.odom = msg

    def test_drive_forward(self):
        linear_x = 0.400
        angular_z = 0.0
        secs = 1

        world_x_exp = 0.400
        world_y_exp = 0.0
        world_theta_exp = 0.0

        self._drive(
            linear_x, angular_z, secs,
            world_x_exp, world_y_exp, world_theta_exp
        )

    def test_drive_reverse(self):
        linear_x = -0.400
        angular_z = 0.0
        secs = 1

        world_x_exp = -0.400
        world_y_exp = 0.0
        world_theta_exp = 0.0

        self._drive(
            linear_x, angular_z, secs,
            world_x_exp, world_y_exp, world_theta_exp
        )

    def test_turn_left(self):
        linear_x = 0.0
        angular_z = math.pi / 4
        secs = 2

        world_x_exp = 0.0
        world_y_exp = 0.0
        world_theta_exp = math.pi / 2

        self._drive(
            linear_x, angular_z, secs,
            world_x_exp, world_y_exp, world_theta_exp
        )

    def test_turn_right(self):
        linear_x = 0.0
        angular_z = -math.pi / 4
        secs = 2

        world_x_exp = 0.0
        world_y_exp = 0.0
        world_theta_exp = -math.pi / 2

        self._drive(
            linear_x, angular_z, secs,
            world_x_exp, world_y_exp, world_theta_exp
        )

    def test_multi_movement(self):
        # self.test_drive_reverse()
        self.test_drive_forward()
        self.test_turn_left()
        self.test_drive_forward()
        self.test_turn_right()
        self.test_drive_reverse()
        self.test_turn_right()
        self.test_drive_forward()
        self.test_turn_left()

    def _drive(self, linear_x, angular_z, secs,
               world_x_exp, world_y_exp, world_theta_exp):
        print()
        print("Command: x:{}, z:{} for {} secs".format(linear_x, angular_z, secs))

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        loops = secs * self._loop_hz
        looprate = rospy.Rate(self._loop_hz)

        start_world_x = self.odom.pose.pose.position.x
        start_world_y = self.odom.pose.pose.position.y
        start_world_theta = yaw_from_odom_message(self.odom)

        print("Expected Relative World: ({}, {}, {})".format(
            world_x_exp, world_y_exp, world_theta_exp))
        print("Starting World: ({}, {}, {})".format(
            start_world_x, start_world_y, start_world_theta))

        # Run for "secs" seconds
        for i in range(loops):
            print("[ODOM] ({}, {}, {}) at {}".format(
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y,
                yaw_from_odom_message(self.odom),
                dt.now()
            ))
            self.pub_cmd_vel.publish(cmd)
            looprate.sleep()

        # Issue stop command
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)

        rospy.sleep(1)
        self._compare_odometry(self.odom, world_x_exp, world_y_exp, world_theta_exp,
                               start_world_x, start_world_y, start_world_theta)

    def _compare_odometry(self, odom, world_x_exp, world_y_exp, world_theta_exp,
                          start_world_x, start_world_y, start_world_theta):
        """Compares the expected world coordinates with an Odometry message
        Parameters:
            :param Odometry odom: The Odometry message
            :param float world_x_exp: The expected world X coordinate
            :param float world_y_exp: The expected world Y coordinate
            :param float world_theta_exp: The expected world Theta orientation
            :param float start_world_x: The staring world X coordinate
            :param float start_world_y: The staring world Y coordinate
            :param float start_world_theta: The staring world Theta coordinate
        """
        # print("Staring World: ({}, {}, {})".format(
        #     start_world_x, start_world_y, start_world_theta))
        # print("Expected World: ({}, {}, {})".format(world_x_exp, world_y_exp, world_theta_exp))

        x_actual = odom.pose.pose.position.x
        y_actual = odom.pose.pose.position.y
        th_actual = tf.transformations.euler_from_quaternion(
            [
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ])[2]
        print("Actual World/Odom: ({}, {}, {})".format(x_actual, y_actual, th_actual))

        # Adjust expected pose to accomodate the starting pose using a 2D transformation
        x_exp_adj, y_exp_adj, theta_exp_adj = transform_pose_2d(
            world_x_exp, world_y_exp, world_theta_exp,
            start_world_x, start_world_y, start_world_theta)

        print("Actual: ({}, {}, {}), Expected: ({}, {}, {})".format(
            x_actual, y_actual, th_actual, x_exp_adj, y_exp_adj, theta_exp_adj))
        self.assertAlmostEqual(x_actual, x_exp_adj, places=2)
        self.assertAlmostEqual(y_actual, y_exp_adj, places=2)
        self.assertAlmostEqual(th_actual, theta_exp_adj, places=2)


def transform_pose_2d(x, y, theta, x_offset, y_offset, theta_offset):
    """Performs a 2D tranformation of a pose (x, y, theta) by transforming
    it by an offset (x_offset, y_offset, theta_offset).

    Parameters:
        :param float x: The x to transform
        :param float y: The y to transform
        :param float theta: The theta to transform
        :param float x_offset: The x offset to transform by
        :param float y_offset: The y offset to transform by
        :param float theta_offset: The theta offset to transform by

    Returns: Tuple representing the transformed pose (x, y, theta)
        :rtype: (float, float, float)
    """
    # First rotate
    x_rotated = (x * math.cos(theta_offset) - y * math.sin(theta_offset))
    y_rotated = (x * math.sin(theta_offset) + y * math.cos(theta_offset))
    print("Rotated XY: ({}, {})".format(x_rotated, y_rotated))

    # Then translate
    x_translated = x_rotated + x_offset
    y_translated = y_rotated + y_offset
    print("Translated XY: ({}, {})".format(x_translated, y_translated))

    # Finally adjust orientation
    theta_new = add_radians(theta, theta_offset)
    print ("Adjusted Expected Theta: {}".format(theta_new))

    return (x_translated, y_translated, theta_new)


def add_radians(a, b):
    """Adds two radian angles, and adjusts sign +/- to be closest to 0
    Parameters:
        :param float a: Angle a
        :param float b: Angle b
    Returns: The resulting angle in radians, with sign adjusted
        :rtype: float
    """
    return (a + b + math.pi) % (2 * math.pi) - math.pi


if __name__ == '__main__':

    import rostest
    rostest.rosrun(PKG, NAME, TestBaseNode)
