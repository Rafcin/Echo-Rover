#!/usr/bin/env python
from __future__ import print_function
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

DEFAULT_NODE_NAME = "teleop_node"

# Publishes
DEFAULT_CMD_TOPIC = "base_node/cmd_vel"

DEFAULT_PUB_HZ = 10
DEFAULT_MAX_LINEAR_SPEED = 0.1  # m/sec
DEFAULT_MAX_ANGULAR_SPEED = 1.0  # radians/sec
X_AXIS = 4
Y_AXIS = 3


class TeleOpNode:
    def __init__(self, node_name):
        self._node_name = node_name
        self._max_linear_speed = rospy.get_param("~max_linear", DEFAULT_MAX_LINEAR_SPEED)
        self._max_angular_speed = rospy.get_param("~max_angular", DEFAULT_MAX_ANGULAR_SPEED)

        self._x_pct = 0.0
        self._z_pct = 0.0
        self._lock = threading.RLock()  # Lock to serialize access to x_pct and z_pct

        # Set up the Joy message Subscriber
        rospy.Subscriber("joy", Joy, self._joy_callback)

        # Set up the Twist publisher
        self._cmd_vel_pub = rospy.Publisher(
            rospy.get_param('~cmd_vel', DEFAULT_CMD_TOPIC),
            Twist,
            queue_size=1
        )

    def run(self):
        """Runs the main loop of the node.
        Publishes the Twist message at a rate of ~cmd_pub_hz.
        """
        rospy.loginfo("Running node")
        looprate = rospy.Rate(rospy.get_param("~cmd_pub_hz", DEFAULT_PUB_HZ))

        try:
            while not rospy.is_shutdown():
                cmd = Twist()
                with self._lock:
                    cmd.linear.x = self._max_linear_speed * self._x_pct
                    cmd.angular.z = self._max_angular_speed * self._z_pct
                self._cmd_vel_pub.publish(cmd)
                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def _joy_callback(self, msg):
        """Called by the Joy message subscriber.
        Converts the Joy input percentages into a Twist velocity message
        and publishes on the <node_name>/cmd_vel topic.

        Parameters:
            :param Joy msg: The Joy topic message
        """
        with self._lock:
            self._x_pct = msg.axes[X_AXIS]
            self._z_pct = msg.axes[Y_AXIS]


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    node = TeleOpNode(node_name)
    node.run()
