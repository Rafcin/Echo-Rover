from __future__ import print_function
import threading
import math
from numpy import sign, clip

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from b2_logic.odometry_helpers import heading_from_odometry, normalize_theta, calc_steering_angle

# Mode enum
MODE_FORWARD = 0
MODE_OBSTACLE_PLAN = 1
MODE_OBSTACLE_TURN = 2


class PilotNode:
    def __init__(self, loophz, turn_radians, turn_radians_tolerance, cmd_vel_pub, pcontroller):
        """
        Parameters:
            :param int loophz:
            :param float turn_radians:
            :param float turn_radians_tolerance:
            :param rospy.Publisher cmd_vel_pub:
            :param PContoller pcontroller:
        """
        self._loophz = loophz
        self._turn_radians = turn_radians
        self._turn_radians_tolerance = turn_radians_tolerance
        self._cmd_vel_pub = cmd_vel_pub
        self._pcontroller = pcontroller

        self._prox_sensor = False
        self._odom = Odometry()
        self._state_lock = threading.RLock()

        self._current_heading = 0.0      # radians
        self._mode = MODE_OBSTACLE_PLAN  # MODE_XXX enum
        self._heading_goal = 0.0         # radians
        self._obstacle_forward = None    # True/False/None
        self._obstacle_right = None      # True/False/None
        self._obstacle_left = None       # True/False/None
        self._reverse_plan = False       # True/False

    def run(self):
        looprate = rospy.Rate(self._loophz)
        try:
            while not rospy.is_shutdown():

                # Update the heading state
                with self._state_lock:
                    self._current_heading = heading_from_odometry(self._odom)
                    rospy.logdebug(
                        "Current heading: {} deg (goal: {} deg)".format(
                            round(math.degrees(normalize_theta(self._current_heading)), 2),
                            round(math.degrees(self._heading_goal)), 2))

                self._decide()
                looprate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    # ~~~~~~~~~~~~~~~~~~~~~~~~
    #  Subscription callbacks
    # ~~~~~~~~~~~~~~~~~~~~~~~~
    def prox_callback(self, msg):
        """
            :param Proximity msg: The Proximity message
        """
        with self._state_lock:
            self._prox_sensor = msg.sensors[0]

    def odom_callback(self, msg):
        """
            :param Odometry msg: The Odometry message
        """
        with self._state_lock:
            self._odom = msg

    # ~~~~~~~~~~~~~~~~~~~~
    #  Non-public methods
    # ~~~~~~~~~~~~~~~~~~~~
    def _send_drive_cmd(self, speed):
        """ Sends the Twist command for linear.x speed in meters/sec
            :param float speed: Speed in meters/sec for linear.x
        """
        cmd = Twist()
        cmd.linear.x = speed
        self._cmd_vel_pub.publish(cmd)

    def _send_turn_cmd(self, radians_sec):
        """ Sends the Twist command for angular.z speed in radians/sec
        :param float radians_sec: Angular speed in radians/sec for angular.z
        """
        cmd = Twist()
        cmd.angular.z = radians_sec
        self._cmd_vel_pub.publish(cmd)
        rospy.logdebug("sent cmd_vel: {}".format(cmd.angular.z))

    def _set_forward_mode(self):
        self._obstacle_forward = None
        self._obstacle_right = None
        self._obstacle_left = None
        self._reverse_plan = False
        self._mode = MODE_FORWARD

    def _decide(self):
        if self._mode == MODE_FORWARD:
            if self._prox_sensor is True:
                # If driving forward, and center sensor detects obstacle
                # --> stop and enter obstacle mode
                self._send_drive_cmd(0)
                self._mode = MODE_OBSTACLE_PLAN
                rospy.logdebug("Obstacle detected while moving forward")
            else:
                # No obstacle, so command base forward some more
                linear_v = self._pcontroller.linear_velocity()
                self._send_drive_cmd(linear_v)
                rospy.logdebug("Forward is clear, proceeding to drive forward")

        else:  # Mode is either _PLAN or _TURN

            # Need to calculate the heading to which to turn next
            if self._mode == MODE_OBSTACLE_PLAN:
                rospy.logdebug("Planning next movement")
                self._process_obstacle_plan()

            # Execute the turn to the target heading
            if self._mode == MODE_OBSTACLE_TURN:
                rospy.logdebug("Turning base")
                self._process_obstacle_turn()

    def _process_obstacle_plan(self):
        """
        Note, the logic here assumes that if self._obstacle_XXX is None
        then we haven't yet been in position to test it. Once we test that
        position, we set the value to either True (obstacle) or False (clear)
        then calculate the turn and switch into TURN mode.

        Therefore, if we are in PLAN mode, we can determine which sides we need
        to test still by examiming the self._obstacle_XXX state.

        Example:
           If in PLAN mode and self._obstacle_forward is NONE, we need to
              test the front position, and if TRUE, turn to the right side.
           If in PLAN mode and self._obstacle_forward is TRUE,
              and self._obstacle_right is NONE: we have turned to the right
              but have not yet tested the right side for an obstacle. So test
              the position and if TRUE, we need to turn to the left side.
        """

        if self._obstacle_forward in (None, False):

            if self._prox_sensor is True:
                # Calculate the turn to check the right side
                self._obstacle_forward = True
                rospy.logdebug("(Planner) Forward is blocked")

                self._heading_goal = normalize_theta(
                    self._current_heading - self._turn_radians)

                rospy.logdebug(
                    "(Planner) Turning to check right side. New heading: {}".format(
                        math.degrees(self._heading_goal)))
                self._mode = MODE_OBSTACLE_TURN
            else:
                self._set_forward_mode()

        elif self._obstacle_right is None:
            if self._prox_sensor is True:
                # Calculate the turn to check the left side
                # We've already turned to the right, so we need to turn 180 to test
                # the left side
                self._obstacle_right = True
                rospy.logdebug("(Planner) Right side is blocked")
                self._heading_goal = normalize_theta(
                    self._current_heading + self._turn_radians * 2)
                rospy.logdebug("(Planner) Turning to check left side. New heading: {}".format(
                        math.degrees(self._heading_goal)))
                self._mode = MODE_OBSTACLE_TURN
            else:
                self._set_forward_mode()

        elif self._obstacle_left is None:
            if self._prox_sensor is True:
                # All three of fwd, right, left are blocked
                self._obstacle_left = True
                rospy.logdebug("(Planner) left is blocked")
                self._heading_goal = normalize_theta(
                    self._current_heading + self._turn_radians)
                rospy.logdebug("(Planner) Turning to rear to backtrack. New heading: {}".format(
                        math.degrees(self._heading_goal)))
                self._mode = MODE_OBSTACLE_TURN
                self._reverse_plan = True
            else:
                self._set_forward_mode()

        elif self._reverse_plan is True:
            # We were performing a turn to reverse. Since we're in plan mode
            # again, this means the turn is complete
            rospy.logdebug("(Planner) Turn to rear complete, moving forward")
            self._set_forward_mode()

        else:
            # This should not be possible
            message = "Obstacle plan logic reached else block that should not be possible"
            rospy.logerr(message)
            raise RuntimeError(message)

    def _process_obstacle_turn(self):
        steering_angle = calc_steering_angle(self._current_heading, self._heading_goal)
        rospy.logdebug("Steering angle: {} radians".format(round(steering_angle, 2)))
        if abs(steering_angle) > self._turn_radians_tolerance:
            # We still need to turn some more
            angular_v = self._pcontroller.angular_velocity(
                self._current_heading, self._heading_goal)
            self._send_turn_cmd(angular_v)

        else:
            # We are done turning, back to obstacle planning
            self._mode = MODE_OBSTACLE_PLAN
            rospy.logdebug(
                "Turn is complete (delta {} < turn radians tolerance {})".format(
                    steering_angle, self._turn_radians_tolerance))


class PVelocityController:
    def __init__(self, min_linear_v, max_linear_v,
                 min_angular_v, max_angular_v, linear_k=1, angular_k=1):
        self.min_linear_v = min_linear_v
        self.max_linear_v = max_linear_v
        self.max_angular_v = max_angular_v
        self.min_angular_v = min_angular_v
        self.linear_k = linear_k
        self.angular_k = angular_k

    def linear_velocity(self, distance_m=1):
        """Calculat the linear velocity using a Proportional (P) method,
        clamped to within the min and max linear speeds.
        Parameters:
            :param float distance_m: Distance to drive
        Returns:
            The linear velocity in m/sec
            :rtype: float
        """
        linear_v = self.linear_k * distance_m
        _sign = sign(linear_v)
        return clip(linear_v, self.min_linear_v, self.max_linear_v) * _sign

    def angular_velocity(self, current_angle, target_angle):
        """Calculate the angular velocity using a Proportional (P) method,
        clamped to within the min and max angular speeds.
        Parameters:
            :param float current_angle: The current heading of the robot in radians
            :param float target_angle: The goal heading of the robot in radians
        Returns:
            The angular velocity in radians/sec
            :rtype: float
        """
        angular_v = self.angular_k * calc_steering_angle(current_angle, target_angle)
        _sign = sign(angular_v)
        return clip(abs(angular_v), self.min_angular_v, self.max_angular_v) * _sign
