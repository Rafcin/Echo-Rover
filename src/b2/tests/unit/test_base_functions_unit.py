#!/usr/bin/env python
from __future__ import print_function
import unittest
import math

import rospy

from b2_logic.base_functions import (
    calc_create_speed_cmd,
    _calc_qpps,
    _calc_wheel_angular_velocity,
    _calc_wheel_linear_velocity,
    _calc_base_frame_velocity,
    calc_base_frame_velocity_from_encoder_diffs,
    calc_odometry_from_base_velocity
)
from b2_logic.odometry_helpers import (
    yaw_from_odom_message, heading_from_odometry, calc_world_frame_velocity
)

PKG = 'b2'
NAME = 'b2_base_functions_unittest'


class TestBaseFunctions(unittest.TestCase):

    def setUp(self):
        print()

        # Robot parameters
        self.wheel_dist = 0.220
        self.wheel_radius = 0.0325
        # self.ticks_per_radian = 48 * 34
        self.ticks_per_rotation = 1632
        self.base_frame_id = "base_frame"
        self.world_frame_id = "world_frame"
        self.max_drive_secs = 1
        self.max_qpps = 6000

    def test_calc_speed_command(self):

        tests = [
            # (x_linear, z_angular, m1_expected, m2_expected)
            # x is m/sec, and z is radians/sec
            (0.1, 0.0, 799, 799),
            (0.0, 0.1, 88, -88),
            (0.05, 0.5, 839, -40),
            (1.0, 0.0, self.max_qpps, self.max_qpps),
            (0.0, math.pi/4, 690, -690),
            (0.5, 0.0, 3996, 3996),
        ]
        print()

        for x_linear_cmd, z_angular_cmd, m1_expected, m2_expected in tests:
            actual_cmd = calc_create_speed_cmd(
                x_linear_cmd,
                z_angular_cmd,
                self.wheel_dist,
                self.wheel_radius,
                self.ticks_per_rotation,
                self.max_drive_secs,
                self.max_qpps
            )
            print()
            print("### Input x_linear: {}, z_angular: {} ###".format(
                x_linear_cmd, z_angular_cmd))
            print("[m1_qpps] Actual: {}, Expected: {}".format(
                actual_cmd.m1_qpps, m1_expected))
            print("[m2_qpps] Actual: {}, Expected: {}".format(
                actual_cmd.m2_qpps, m2_expected))
            print("[max_secs] Actual: {}, Expected: {}".format(
                actual_cmd.max_secs, self.max_drive_secs))

            self.assertEqual(round(actual_cmd.m1_qpps), m1_expected)
            self.assertEqual(round(actual_cmd.m2_qpps), m2_expected)
            self.assertEqual(round(actual_cmd.max_secs), self.max_drive_secs)

    def test_calc_odometry_single(self):
        """Calculates the change in odometry after a single movement

            M1 is the right motor, M2 is the left motor
        """

        # world_x, world_y, world_theta, last_odom_time, m1_qpps, m2_qpps, delta_secs
        # new_world_x, new_world_y, new_world_theta, exp_linear_x, exp_linear_y, exp_angular_z
        tests = [
            # Drive straight at 1000 QPPS
            (0.0, 0.0, 0.0, rospy.Time(0), 1000, 1000, 1,
                0.125, 0.0, 0.0, 0.125, 0.0, 0.0),

            # Turn left and forward
            (0.0, 0.0, 0.0, rospy.Time(0), 1000, 500, 1,
                0.094, 0.0, 0.284, 0.094, 0.0, 0.284),

            # Drive straight at 3700 QPPS
            (0.0, 0.0, 0.0, rospy.Time(0), 3700, 3700, 1,
                0.463, 0.0, 0.0, 0.463, 0.0, 0.0),

            # Drive right and forward, starting not at the origin
            (3.0, 4.0, math.pi/4, rospy.Time(0), 700, 2800, 0.5,
                3.077, 4.077, 0.188, 0.155, 0.155, -1.194),

            # Same as above, but starting at a non-zero time
            (3.0, 4.0, math.pi/4, rospy.Time(12000), 700, 2800, 0.5,
                3.077, 4.077, 0.188, 0.155, 0.155, -1.194),

            # # Rotate right 90 degrees
            # (0.0, 0.0, 0.0, rospy.Time(0), 1000, 1000, 1,
            #     0.125, 0.0, 0.0, 0.125, 0.0, 0.0),
        ]

        for (
            world_x, world_y, world_theta,
            last_odom_time,
            m1_qpps, m2_qpps, delta_secs,
            exp_new_world_x, exp_new_world_y, exp_new_world_theta,
            exp_linear_x, exp_linear_y, exp_angular_z
        ) in tests:
            print()
            print("#### Input: m1: {}, m2: {}, w({}, {}, {}), secs: {}".format(
                m1_qpps, m2_qpps, world_x, world_y, world_theta, delta_secs))
            t2 = last_odom_time + rospy.Duration(secs=delta_secs)

            m1_enc_diff = m1_qpps * delta_secs
            m2_enc_diff = m2_qpps * delta_secs

            x_linear_v, y_linear_v, z_angular_v = calc_base_frame_velocity_from_encoder_diffs(
                m1_enc_diff, m2_enc_diff, self.ticks_per_rotation,
                self.wheel_radius, self.wheel_dist,
                last_odom_time, t2
            )
            print("Base velocities ({}, {}, {})".format(x_linear_v, y_linear_v, z_angular_v))

            odom = calc_odometry_from_base_velocity(
                x_linear_v, y_linear_v, z_angular_v,
                world_x, world_y, world_theta,
                (t2 - last_odom_time).to_sec(), t2,
                self.base_frame_id, self.world_frame_id
            )

            self._compare_odometry(odom, exp_new_world_x, exp_new_world_y, exp_new_world_theta,
                                   exp_linear_x, exp_linear_y, exp_angular_z)

    def test_calc_odometry_cumulative(self):
        # (linear_x, angular_z, secs,
        #  exp_world_x, exp_world_y, exp_world_theta, exp_linear_x, exp_linear_y, exp_angular_z)
        tests = [
            # Straight for 1 meter
            (0.5, 0.0, 2,
                1.0, 0.0, 0.0, 0.5, 0.0, 0.0),
            # Turn 90 degrees
            (0.0, math.pi/4, 2,
                1.0, 0.0, math.pi/2, 0.0, 0.0, math.pi/4),
            # Straight for 1 meter
            (0.5, 0.0, 2,
                1.0, 1.0, math.pi/2, 0.0, 0.5, 0.0),
            # Turn 135 degrees
            (0.0, math.pi/4, 3,
                1.0, 1.0, -math.pi/4*3, 0.0, 0.0, math.pi/4),
            # Straight back to origin, Not 0,0 because of 0.1 sec odom update
            (0.5, 0.0, 2 * (1 / math.sin(math.pi/4)),
                0.01, 0.01, -math.pi/4*3, -0.354, -0.354, 0.0),
        ]

        world_x = 0.0
        world_y = 0.0
        world_theta = 0.0
        last_odom_time = rospy.Time(0)
        delta_secs = 0.1

        print()

        for (linear_x, angular_z, secs,
             exp_world_x, exp_world_y, exp_world_theta,
             exp_linear_x, exp_linear_y, exp_angular_z) in tests:

            print()
            print("CMD [x:{}, w:{}] for {} secs".format(linear_x, angular_z, secs))

            for i in range(int(secs / delta_secs)):
                t2 = last_odom_time + rospy.Duration(secs=delta_secs)

                cmd = calc_create_speed_cmd(
                    linear_x, angular_z,
                    self.wheel_dist, self.wheel_radius, self.ticks_per_rotation,
                    self.max_drive_secs, self.max_qpps
                )

                print("M1 QPPS: {} M2 QPPS: {}".format(cmd.m1_qpps, cmd.m2_qpps))

                m1_enc_diff = cmd.m1_qpps * delta_secs
                m2_enc_diff = cmd.m2_qpps * delta_secs

                x_linear_v, y_linear_v, z_angular_v = calc_base_frame_velocity_from_encoder_diffs(
                    m1_enc_diff, m2_enc_diff, self.ticks_per_rotation,
                    self.wheel_radius, self.wheel_dist,
                    last_odom_time, t2
                )

                odom = calc_odometry_from_base_velocity(
                    x_linear_v, y_linear_v, z_angular_v,
                    world_x, world_y, world_theta,
                    (t2 - last_odom_time).to_sec(), t2,
                    self.base_frame_id, self.world_frame_id
                )

                world_x = odom.pose.pose.position.x
                world_y = odom.pose.pose.position.y
                world_theta = yaw_from_odom_message(odom)
                last_odom_time = t2
                print("x: {}, y: {}, 0: {}, t2: {}".format(
                    world_x, world_y, world_theta, t2))

            self._compare_odometry(
                odom,
                exp_world_x, exp_world_y, exp_world_theta,
                exp_linear_x, exp_linear_y, exp_angular_z
            )

    def _compare_odometry(self, actual_odom, exp_new_world_x, exp_new_world_y, exp_new_world_theta,
                          exp_linear_x, exp_linear_y, exp_angular_z):
        new_world_x = round(actual_odom.pose.pose.position.x, 3)
        new_world_y = round(actual_odom.pose.pose.position.y, 3)

        new_world_theta = yaw_from_odom_message(actual_odom)

        new_world_linear_x = round(actual_odom.twist.twist.linear.x, 3)
        new_world_linear_y = round(actual_odom.twist.twist.linear.y, 3)
        new_world_angular_z = round(actual_odom.twist.twist.angular.z, 3)

        print("World Pose Actual: [{}, {}, {}], Expected: [{}, {}, {}]".format(
            new_world_x, new_world_y, new_world_theta,
            exp_new_world_x, exp_new_world_y, exp_new_world_theta))
        print("World Velocity Actual: [{}, {}, {}], Expected: [{}, {}, {}]".format(
            new_world_linear_x, new_world_linear_y, new_world_angular_z,
            exp_linear_x, exp_linear_y, exp_angular_z))

        self.assertAlmostEqual(new_world_x, exp_new_world_x, places=2)
        self.assertAlmostEqual(new_world_y, exp_new_world_y, places=2)
        self.assertAlmostEqual(new_world_theta, exp_new_world_theta, places=2)
        self.assertAlmostEqual(new_world_linear_x, exp_linear_x, places=2)
        self.assertAlmostEqual(new_world_linear_y, exp_linear_y, places=2)
        self.assertAlmostEqual(new_world_angular_z, exp_angular_z, places=2)

    def test_calc_qpps(self):
        # (m1_enc_diff, m2_enc_diff, delta_secs,
        #  exp_m1_qpps, exp_m2_qpps)
        tests = [
            # Simplest case
            (1000, 1000, 1,
             1000, 1000),
            # Positive, Negative case
            (300, -300, 0.1,
             3000, -3000),
            # Fractional QPPS case (+/-)
            (100, -100, 0.3,
             333.33, -333.33)
        ]

        for (
            m1_enc_diff, m2_enc_diff, delta_secs,
            exp_m1_qpps, exp_m2_qpps
        ) in tests:
            actual_m1_qpps, actual_m2_qpps = _calc_qpps(m1_enc_diff, m2_enc_diff, delta_secs)
            print()
            print("M1 enc diff: {}, M2 enc diff: {}, Secs: {}".format(
                m1_enc_diff, m2_enc_diff, delta_secs))
            print("M1 QPPS - actual: {}, expected: {} | M2 QPPS - actual: {}, expected: {}".format(
                actual_m1_qpps, exp_m1_qpps, actual_m2_qpps, exp_m2_qpps))
            self.assertAlmostEqual(actual_m1_qpps, exp_m1_qpps, 0)
            self.assertAlmostEqual(actual_m2_qpps, exp_m2_qpps, 0)

    def test_calc_wheel_angular_velocity(self):
        """ M1 is right motor, M2 is left motor
        """

        # (m1_qpps, m2_qpps, ticks_per_rotatation,
        #  exp_angular_right_v, exp_angular_left_v)
        tests = [
            # Simple case
            (1000, 1000, 1632,
             3.850, 3.850),
            # Postive/Negative case
            (-1000, 1000, 1632,
             -3.850, 3.850),
            # Zero QPPS case
            (-0, 0, 1632,
             0.0, 0.0),
            # Zero ticks/rotation case (this should not happen - div/zero error)
            (1000, -1000, 0,
             0.0, 0.0),
        ]

        for (
            m1_qpps, m2_qpps, ticks_per_rotatation, exp_angular_right_v, exp_angular_left_v
        ) in tests:
            left_angular_v, right_angular_v = _calc_wheel_angular_velocity(
                m1_qpps, m2_qpps, ticks_per_rotatation)
            print()
            print("M1 (right) QPPS: {}, M2 (left) QPPS: {}, Ticks/rotation: {}".format(
                m1_qpps, m2_qpps, ticks_per_rotatation))
            print("Right actual: {}, expected: {} | Left actual: {}, expected: {}".format(
                right_angular_v, exp_angular_right_v, left_angular_v, exp_angular_left_v))
            self.assertAlmostEqual(right_angular_v, exp_angular_right_v, 3)
            self.assertAlmostEqual(left_angular_v, exp_angular_left_v, 3)

    def test_calc_wheel_linear_velocity(self):
        # (left_angular_v, right_angular_v, wheel_radius,
        #  exp_left_linear_v, exp_right_linear_v)
        tests = [
            # Simplifed case
            (1.0, 1.0, 0.1,
             0.1, 0.1),
            # Realistic case
            (11.21, 11.34, 0.0325,
             0.364325, 0.36855),
            # Positive/Negative case
            (11.21, -11.34, 0.0325,
             0.364325, -0.36855),
            # Zero angular_v case
            (11.21, 0.0, 0.0325,
             0.364325, 0.0),
            # Zero wheel_radius case (this should not happen as wheels have a radius)
            # Return zero which is computaionally correct, but also log error
            (11.21, 11.34, 0.0,
             0.0, 0.0),
            # Negative wheel_radius case (this should not happen also)
            # Return zero linear_v, and also log error
            (11.21, 11.34, -0.0325,
             0.0, 0.0),
        ]

        for (
            left_angular_v, right_angular_v, wheel_radius, exp_left_linear_v, exp_right_linear_v
        ) in tests:
            actual_left_linear_v, actual_right_linear_v = _calc_wheel_linear_velocity(
                left_angular_v, right_angular_v, wheel_radius)
            print()
            print("Right angular_v: {}, left: {}, wheel_radius: {}".format(
                right_angular_v, left_angular_v, wheel_radius))
            print("Right linear_v actual: {}, expected: {} | Left actual: {}, expected: {}".format(
                actual_right_linear_v, exp_right_linear_v,
                actual_left_linear_v, exp_left_linear_v)
            )
            self.assertAlmostEqual(actual_right_linear_v, exp_right_linear_v, 3)
            self.assertAlmostEqual(actual_left_linear_v, exp_left_linear_v, 3)

    def test_calc_base_frame_velocity(self):
        # (left_linear_v, right_linear_v, wheel_dist,
        #  exp_x_linear_v, exp_y_linear_v, exp_z_angular_v)
        tests = [
            # Straight forward case
            (1.0, 1.0, 0.1,
             1.0, 0.0, 0.0),
            # Rotate right in place case
            (0.1, -0.1, 0.1,
             0.0, 0.0, -2.0),
            # Rotate left in place case
            (-0.1, 0.1, 0.1,
             0.0, 0.0, 2.0),
            # Right turn circle case
            (1.0, 0.5, 0.1,
             0.75, 0.0, -5.0),
            # Left turn circle case
            (0.5, 1.0, 0.1,
             0.75, 0.0, 5.0),
        ]

        for (
            left_linear_v, right_linear_v, wheel_dist,
            exp_x_linear_v, exp_y_linear_v, exp_z_angular_v
        ) in tests:
            x_linear_v, y_linear_v, z_angular_v = _calc_base_frame_velocity(
                left_linear_v, right_linear_v, wheel_dist)
            print()
            print("Left linear_v: {}, right linear_v: {}, wheel dist: {}".format(
                left_linear_v, right_linear_v, wheel_dist))
            print("X linear_v actual: {}, expected: {} | Y linear_v actual: {},"
                  "expected: {} | Z angular actual: {}, expected: {}".format(
                    x_linear_v, exp_x_linear_v, y_linear_v, exp_y_linear_v,
                    z_angular_v, exp_z_angular_v))
            self.assertAlmostEqual(x_linear_v, exp_x_linear_v, 3)
            self.assertAlmostEqual(y_linear_v, exp_y_linear_v, 3)
            self.assertAlmostEqual(z_angular_v, exp_z_angular_v, 3)

    def test_calc_world_frame_velocity(self):
        # (x_linear_v, y_linear_v, z_angular_v, world_theta,
        #  exp_world_x_velocity, exp_world_y_velocity, exp_world_angular_velocity)
        tests = [
            # Forward 1.0 m/s, no rotation, from 0-deg heading
            (1.0, 0.0, 0.0, 0.0,
             1.0, 0.0, 0.0),
            # Forward 1.0 m/s, no rotation, 90-deg left heading
            (1.0, 0.0, 0.0, math.pi / 2,
             0.0, 1.0, 0.0),
            # Forward 1.0 m/s, rotation 90-deg left, from 0-deg heading
            (1.0, 0.0, 1.0, 0.0,
             1.0, 0.0, 1.0),
            # Forward 1.0 m/s, rotation 90-deg right, from 90-left heading
            (1.0, 0.0, -1.0, math.pi / 2,
             0.0, 1.0, -1.0),
            # Forward 1.0 m/s, no rotation, from 45-deg left heading
            (1.0, 0.0, 0.0, math.pi / 4,
             0.707, 0.707, 0.0),
            # Positive base velocity, but negative world velocity because of 270-deg heading
            (1.0, 0.0, 0.0, 2 * math.pi * 3/4,  # 90-deg to the right
             0.0, -1.0, 0.0),
        ]

        for (
            x_linear_v, y_linear_v, z_angular_v, world_theta,
            exp_world_x_velocity, exp_world_y_velocity, exp_world_z_velocity
        ) in tests:

            (world_x_velocity, world_y_velocity,
                world_z_velocity) = calc_world_frame_velocity(
                    x_linear_v, y_linear_v, z_angular_v, world_theta)
            print()
            print("Inputs x_linear_v: {}, y_linear_v: {}, z_angular_v: {}, world_theta: {}".format(
                x_linear_v, y_linear_v, z_angular_v, world_theta))
            print("World X velocity, actual: {}, expected: {}".format(
                world_x_velocity, exp_world_x_velocity))
            print("World Y velocity, actual: {}, expected: {}".format(
                world_y_velocity, exp_world_y_velocity))
            print("World Z velocity, actual: {}, expected: {}".format(
                world_z_velocity, exp_world_z_velocity))
            self.assertAlmostEqual(world_x_velocity, exp_world_x_velocity, 3)
            self.assertAlmostEqual(world_y_velocity, exp_world_y_velocity, 3)
            self.assertAlmostEqual(world_z_velocity, exp_world_z_velocity, 3)

    def test_calc_base_frame_velocity_from_encoder_diffs(self):
        # (m1_enc_diff, m2_enc_diff, ticks_per_rotation, wheel_radius, wheel_dist, duration_secs,
        #  exp_x_linear_v, exp_y_linear_v, exp_z_angular_v)
        tests = [
            # One wheel distance forward in 1/2 sec
            (1632, 1632, 1632, 0.0325, 0.220, 0.5,
             0.408407045, 0.0, 0.0),
            # One wheel distance forward in 1.0 sec
            (1632, 1632, 1632, 0.0325, 0.220, 1.0,
             0.2042035225, 0.0, 0.0),
            # Rotate in place one wheel distance in 1.0 sec
            (1632, -1632, 1632, 0.0325, 0.220, 1.0,
             0.0, 0.0, 0.2042035225 / (0.220 / 2)),  # wheel roll dist / 1/2 wheel base (R)

        ]

        for (
            m1_enc_diff, m2_enc_diff, ticks_per_rotation, wheel_radius,
            wheel_dist, duration_secs, exp_x_linear_v, exp_y_linear_v, exp_z_angular_v
        ) in tests:
            x_linear_v, y_linear_v, z_angular_v = calc_base_frame_velocity_from_encoder_diffs(
                m1_enc_diff, m2_enc_diff, ticks_per_rotation, wheel_radius, wheel_dist,
                rospy.Time(0), rospy.Time(duration_secs)
            )
            print()
            print("m1_enc_diff: {}, m2_enc_diff: {}, duration_secs: {}".format(
                m1_enc_diff, m2_enc_diff, duration_secs))
            print("wheel_radius: {}, wheel_dist: {}, ticks_per_rotation: {}".format(
                wheel_radius, wheel_dist, ticks_per_rotation))
            print("exp_x_linear_v: {}, exp_y_linear_v: {}, exp_z_angular_v: {}".format(
                exp_x_linear_v, exp_y_linear_v, exp_z_angular_v))
            self.assertAlmostEqual(x_linear_v, exp_x_linear_v, places=3)
            self.assertAlmostEqual(y_linear_v, exp_y_linear_v, places=3)
            self.assertAlmostEqual(z_angular_v, exp_z_angular_v, places=3)

    def test_calc_odometry_from_base_velocity(self):
        # (x_linear_v, y_linear_v, z_angular_v, world_x, world_y, world_theta, duration_secs,
        #  exp_x, exp_y, exp_theta, exp_x_v, exp_y_v, exp_z_v)
        tests = [
            # Straight 1.0 m/s for 1 sec from (0, 0, 0)
            (1.0, 0.0, 0.0, 0, 0, 0, 1,
             1.0, 0.0, 0.0, 1, 0, 0),
            # Reverse 0.5 m/s for 2 sec from (1, 0, 0)
            (-0.5, 0.0, 0.0, 1, 0, 0, 2,
             0.0, 0.0, 0.0, -0.5, 0, 0),
            # Forward 1.0 m/s for 1 sec from (1.5, -1.5, 45-deg)
            (1.0, 0.0, 0.0, 1.5, -1.5, math.pi/4, 1,
             1.5 + 0.707, -1.5 + 0.707, math.pi/4, 0.707, 0.707, 0.0),
        ]

        for (
            x_linear_v, y_linear_v, z_angular_v, world_x, world_y, world_theta, duration_secs,
            exp_x, exp_y, exp_theta, exp_x_v, exp_y_v, exp_z_v
        ) in tests:
            print()
            print("x_lin_v {}, y_lin_v {}, z_ang_v {}, World Pose ({}, {}, {}), "
                  "duration_secs {}".format(
                    x_linear_v, y_linear_v, z_angular_v,
                    world_x, world_y, world_theta, duration_secs))

            odom = calc_odometry_from_base_velocity(
                x_linear_v, y_linear_v, z_angular_v, world_x, world_y, world_theta,
                duration_secs, rospy.Time(10), "base", "world"
            )
            world_x = odom.pose.pose.position.x
            world_y = odom.pose.pose.position.y
            world_theta = heading_from_odometry(odom)
            world_x_v = odom.twist.twist.linear.x
            world_y_v = odom.twist.twist.linear.y
            world_z_v = odom.twist.twist.angular.z

            print("Pose actual ({}, {}, {}), expected ({}, {}, {})".format(
                world_x, world_y, world_theta, exp_x, exp_y, exp_theta))
            self.assertAlmostEqual(world_x, exp_x, places=3)
            self.assertAlmostEqual(world_y, exp_y, places=3)
            self.assertAlmostEqual(world_theta, exp_theta, places=3)

            print("Twist actual ({}, {}, {}), expected ({}, {}, {})".format(
                world_x_v, world_y_v, world_z_v, exp_x_v, exp_y_v, exp_z_v))
            self.assertAlmostEqual(world_x_v, exp_x_v, places=3)
            self.assertAlmostEqual(world_y_v, exp_y_v, places=3)
            self.assertAlmostEqual(world_z_v, exp_z_v, places=3)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestBaseFunctions)
