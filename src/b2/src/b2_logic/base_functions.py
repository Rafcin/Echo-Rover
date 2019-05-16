from math import pi

import rospy

from roboclaw_driver.msg import SpeedCommand
from odometry_helpers import (
    create_odometry_message, calc_world_frame_pose, calc_world_frame_velocity
)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  Below are used by the BaseNode directly
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def calc_create_speed_cmd(x_linear_cmd, z_angular_cmd, wheel_dist,
                          wheel_radius, ticks_per_rotation, max_drive_secs, max_qpps):
    """Calculate and send motor commands

    Parameters:
        :param double x_linear_cmd: Twist message's linear.x value
        :param double z_angular_cmd: Twist message's angular.z value
        :param double wheel_dist: Distance between wheels (m)
        :param double wheel_radius: Wheel radius (m)
        :param double ticks_per_radian: Number of encoder ticks per radian of wheel rotation
        :param double max_drive_secs: Maximum seconds drive should run before stopping

    Returns: The SpeedCommand message
        :rtype: roboclaw.msg.SpeedCommand
    """
    right_angular_v = (
        (x_linear_cmd + z_angular_cmd * (wheel_dist / 2.0)) / wheel_radius
    )
    left_angular_v = (
        (x_linear_cmd - z_angular_cmd * (wheel_dist / 2.0)) / wheel_radius
    )
    # print("left_angular_v: {}".format(left_angular_v))

    ticks_per_radian = ticks_per_rotation / (pi * 2)
    # print("ticks_per_radian: {}".format(ticks_per_radian))

    right_qpps_target = right_angular_v * ticks_per_radian
    left_qpps_target = left_angular_v * ticks_per_radian
    # print("left_qpps_target: {}".format(left_qpps_target))

    # Clamp the target QPPS within the max_qpps
    right_qpps_target = max(-max_qpps, min(right_qpps_target, max_qpps))
    left_qpps_target = max(-max_qpps, min(left_qpps_target, max_qpps))
    # print("right_qpps_target (after clamp): {}".format(right_qpps_target))
    # print("left_qpps_target (after clamp): {}".format(left_qpps_target))

    cmd = SpeedCommand()
    cmd.m1_qpps = right_qpps_target
    cmd.m2_qpps = left_qpps_target
    cmd.max_secs = max_drive_secs
    return cmd


def calc_base_frame_velocity_from_encoder_diffs(
    m1_enc_diff, m2_enc_diff,
    ticks_per_rotation, wheel_radius, wheel_dist,
    begin_odom_time, end_odom_time
):

    # print("begin_odom_time:", begin_odom_time.to_sec())
    # print("end_odom_time:", end_odom_time.to_sec())
    time_delta_secs = (end_odom_time - begin_odom_time).to_sec()
    # print("time_delta_secs: {}".format(time_delta_secs))

    m1_qpps_actual, m2_qpps_actual = _calc_qpps(m1_enc_diff, m2_enc_diff, time_delta_secs)
    # print("m1_qpps_actual: {} / {} = {}".format(m1_enc_diff, time_delta_secs, m1_qpps_actual))
    # print("m2_qpps_actual: {} / {} = {}".format(m2_enc_diff, time_delta_secs, m2_qpps_actual))

    left_angular_v, right_angular_v = _calc_wheel_angular_velocity(
        m1_qpps_actual, m2_qpps_actual, ticks_per_rotation)
    # print("right_angular_v: {}".format(right_angular_v))
    # print("left_angular_v: {}".format(left_angular_v))

    left_linear_v, right_linear_v = _calc_wheel_linear_velocity(
        left_angular_v, right_angular_v, wheel_radius)
    # print("right_linear_v: {}".format(right_linear_v))
    # print("left_linear_v: {}".format(left_linear_v))

    x_linear_v, y_linear_v, z_angular_v = _calc_base_frame_velocity(
        left_linear_v, right_linear_v, wheel_dist)
    return x_linear_v, y_linear_v, z_angular_v


def calc_odometry_from_base_velocity(
    x_linear_v, y_linear_v, z_angular_v,
    world_x, world_y, world_theta,
    time_delta_secs, odom_time,
    base_frame_id, world_frame_id
):
    world_x_velocity, world_y_velocity, world_angular_velocity = calc_world_frame_velocity(
        x_linear_v, y_linear_v, z_angular_v, world_theta)
    # print("world_x_velocity: {}".format(world_x_velocity))
    # print("world_y_velocity: {}".format(world_y_velocity))
    # print("world_angular_velocity: {}".format(world_angular_velocity))

    world_x, world_y, world_theta = calc_world_frame_pose(
        world_x_velocity, world_y_velocity, world_angular_velocity,
        world_x, world_y, world_theta, time_delta_secs)
    # print("world coordinates: ({}, {}, {})".format(world_x, world_y, world_theta))

    odom = create_odometry_message(
        world_x, world_y, world_theta,
        world_x_velocity, world_y_velocity, world_angular_velocity,
        odom_time, base_frame_id, world_frame_id
    )
    return odom


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  Below are used by the functions above
#  Separated out so they can be unit tested
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def _calc_qpps(m1_enc_diff, m2_enc_diff, time_delta_secs):
    if time_delta_secs > 0:
        m1_qpps_actual = m1_enc_diff / float(time_delta_secs)
        m2_qpps_actual = m2_enc_diff / float(time_delta_secs)
    else:
        m1_qpps_actual = m2_qpps_actual = 0
    return (m1_qpps_actual, m2_qpps_actual)


def _calc_wheel_angular_velocity(m1_qpps, m2_qpps, ticks_per_rotation):
    ticks_per_radian = ticks_per_rotation / (2 * pi)
    if ticks_per_radian <= 0.0:
        rospy.logerr("ticks_per_rotation cannot be <= 0.0 (val: {})".format(ticks_per_rotation))
        return (0.0, 0.0)
    right_angular_v = m1_qpps / float(ticks_per_radian)
    left_angular_v = m2_qpps / float(ticks_per_radian)
    return (left_angular_v, right_angular_v)


def _calc_wheel_linear_velocity(left_angular_v, right_angular_v, wheel_radius):
    if wheel_radius <= 0.0:
        wheel_radius = 0.0  # Set it to zero so the linear_v calculates to 0.0
        rospy.logwarn("wheel_radius cannot be <- 0.0 (val: {})".format(wheel_radius))
    right_linear_v = right_angular_v * float(wheel_radius)
    left_linear_v = left_angular_v * float(wheel_radius)
    return (left_linear_v, right_linear_v)


def _calc_base_frame_velocity(left_linear_v, right_linear_v, wheel_dist):
    x_linear_v = (right_linear_v + left_linear_v) / 2.0
    y_linear_v = 0  # Because the robot is nonholonomic
    z_angular_v = (right_linear_v - left_linear_v) / float(wheel_dist)
    return (x_linear_v, y_linear_v, z_angular_v)
