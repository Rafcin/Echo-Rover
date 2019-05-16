#!/usr/bin/env python
from __future__ import print_function

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import tf

from roboclaw_driver.msg import SpeedCommand, Stats
from b2_logic.nodes.base import BaseNode

DEFAULT_NODE_NAME = "base_node"

# Subscribes
DEFAULT_CMD_TOPIC = "base_node/cmd_vel"
DEFAULT_ROBOCLAW_STATS_TOPIC = "roboclaw/stats"

# Publishes
DEFAULT_SPEED_CMD_TOPIC = "roboclaw/speed_command"
DEFAULT_ODOM_TOPIC = "~odom"

DEFAULT_LOOP_HZ = 10  # hertz
DEFAULT_WHEEL_DIST = 0.220  # meters
DEFAULT_WHEEL_RADIUS = 0.0325  # meters
DEFAULT_TICKS_PER_ROTATION = 48 * 34
DEFAULT_MAX_QPPS = 3700
DEFAULT_MAX_DRIVE_SECS = 1
DEFAULT_ODOM_FRAME_ID = "world"
DEFAULT_BASE_FRAME_ID = "base_link"
DEFAULT_DEADMAN_SECS = 1


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()

    wheel_dist = rospy.get_param("~wheel_dist", DEFAULT_WHEEL_DIST)
    wheel_radius = rospy.get_param("~wheel_radius", DEFAULT_WHEEL_RADIUS)
    ticks_per_rotation = rospy.get_param("~ticks_per_rotation", DEFAULT_TICKS_PER_ROTATION)
    max_drive_secs = rospy.get_param("~max_drive_secs", DEFAULT_MAX_DRIVE_SECS)
    deadman_secs = rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SECS)
    max_qpps = rospy.get_param("~max_qpps", DEFAULT_MAX_QPPS)
    base_frame_id = rospy.get_param("~base_frame_id", DEFAULT_BASE_FRAME_ID)
    world_frame_id = rospy.get_param("~odom_frame_id", DEFAULT_ODOM_FRAME_ID)
    loop_hz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)

    # Publishes
    speed_cmd_pub = rospy.Publisher(
        rospy.get_param('~speed_cmd_topic', DEFAULT_SPEED_CMD_TOPIC),
        SpeedCommand,
        queue_size=1
    )
    odom_pub = rospy.Publisher(
        rospy.get_param('~odom_topic', DEFAULT_ODOM_TOPIC),
        Odometry,
        queue_size=1
    )
    tf_broadcaster = tf.broadcaster.TransformBroadcaster()

    node = BaseNode(wheel_dist, wheel_radius, ticks_per_rotation,
                    max_drive_secs, deadman_secs, max_qpps,
                    base_frame_id, world_frame_id,
                    speed_cmd_pub, odom_pub, tf_broadcaster)

    # Joy message Subscriber
    rospy.Subscriber(
        rospy.get_param("~cmd_topic", DEFAULT_CMD_TOPIC),
        Twist,
        node.cmd_vel_callback
    )

    # Roboclaw Stats message Subscriber
    rospy.Subscriber(
        rospy.get_param("~roboclaw_stats_topic", DEFAULT_ROBOCLAW_STATS_TOPIC),
        Stats,
        node.roboclaw_stats_callback
    )

    node.run(loop_hz)
