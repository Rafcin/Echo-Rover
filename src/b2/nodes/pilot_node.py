#!/usr/bin/env python
from __future__ import print_function
import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from b2_logic.nodes.pilot import PilotNode, PVelocityController
from b2.msg import Proximity


DEFAULT_NODE_NAME = "pilot_node"

# Subscribes
DEFAULT_PROXIMITY_TOPIC = "ir_sensors/proximity"
DEFAULT_ODOMETRY_TOPIC = "base_node/odom"

# Publishes
DEFAULT_CMD_TOPIC = "base_node/cmd_vel"

DEFAULT_LOOP_HZ = 5                  # hertz
DEFAULT_MAX_FWD_SPEED = 0.5           # m/sec
DEFAULT_MIN_FWD_SPEED = 0.1           # m/sec
DEFAULT_MAX_TURN_SPEED = math.pi / 4  # radians/sec
DEFAULT_MIN_TURN_SPEED = 0.1          # radians/sec
DEFAULT_TURN_DEGREES = 90             # degrees, will be converted to radians
DEFAULT_TURN_DEGREE_TOLERANCE = 5     # degrees, will be converted to radians
DEFAULT_LINEAR_K = 1
DEFAULT_ANGULAR_K = 1                 # K constant for angular P controller


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)

    node_name = rospy.get_name()
    loophz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)
    max_fwd_speed = rospy.get_param("~max_fwd_speed", DEFAULT_MAX_FWD_SPEED)
    min_fwd_speed = rospy.get_param("~min_fwd_speed", DEFAULT_MIN_FWD_SPEED)
    max_turn_speed = rospy.get_param("~max_turn_speed", DEFAULT_MAX_TURN_SPEED)
    min_turn_speed = rospy.get_param("~min_turn_speed", DEFAULT_MIN_TURN_SPEED)
    turn_radians = math.radians(rospy.get_param("~turn_degrees", DEFAULT_TURN_DEGREES))
    turn_radians_tolerance = math.radians(
        rospy.get_param("~turn_degree_tolerance", DEFAULT_TURN_DEGREE_TOLERANCE))
    linear_k = rospy.get_param("~linear_k", DEFAULT_LINEAR_K)
    angular_k = rospy.get_param("~angular_k", DEFAULT_ANGULAR_K)

    # P-Controller
    pcontroller = PVelocityController(
        min_fwd_speed, max_fwd_speed,
        min_turn_speed, max_turn_speed,
        linear_k=linear_k, angular_k=angular_k
    )

    # Publishes
    cmd_vel_pub = rospy.Publisher(
        rospy.get_param('~cmd_topic', DEFAULT_CMD_TOPIC),
        Twist,
        queue_size=1
    )

    node = PilotNode(loophz, turn_radians, turn_radians_tolerance, cmd_vel_pub, pcontroller)

    # Subscribes
    rospy.Subscriber(
        rospy.get_param("~proximity_topic", DEFAULT_PROXIMITY_TOPIC),
        Proximity,
        node.prox_callback
    )

    # Subscribes
    rospy.Subscriber(
        rospy.get_param("~odom_topic", DEFAULT_ODOMETRY_TOPIC),
        Odometry,
        node.odom_callback
    )

    node.run()
