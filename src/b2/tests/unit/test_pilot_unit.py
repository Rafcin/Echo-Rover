#!/usr/bin/env python
from __future__ import print_function
import unittest
import math

from b2_logic.nodes.pilot import (
    PilotNode, PVelocityController, MODE_FORWARD, MODE_OBSTACLE_PLAN, MODE_OBSTACLE_TURN
)
from b2.msg import Proximity

PKG = 'b2'
NAME = 'b2_pilot_node_unittest'

DEG_90 = math.pi / 2


class PublisherMock:
    def publish(self, msg):
        pass


class TestPilotNode(unittest.TestCase):

    def setUp(self):
        print()
        loophz = 2
        min_fwd_speed = 0.1
        max_fwd_speed = 0.5
        min_turn_speed = 0.1
        max_turn_speed = math.pi / 4
        turn_radians = math.radians(90)
        turn_radians_tolerance = math.radians(5)
        cmd_vel_pub = PublisherMock()

        pcontroller = PVelocityController(
            min_fwd_speed, max_fwd_speed, min_turn_speed, max_turn_speed
        )
        self.node = PilotNode(
            loophz, turn_radians, turn_radians_tolerance, cmd_vel_pub, pcontroller
        )

    def test_fwd_obstacle_flow(self):
        print("Set initial position")
        self.node._current_heading = 0.0
        self._send_proximity_message(False)
        self._print_status()

        print("No obstacle yet, so expect drive forward")
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._current_heading, 0.0)
        self.assertEqual(self.node._mode, MODE_FORWARD)

        print("Detect obstacle")
        self._send_proximity_message(True)
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._current_heading, 0.0)
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)

        print("Decide 90-deg to right")
        right_90 = (2 * math.pi) - DEG_90
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_TURN)
        self.assertEqual(self.node._heading_goal, right_90)

        print("Complete 90-deg right turn")
        self.node._current_heading = right_90
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)

        print("Detect clear, and decide to move forward")
        self._send_proximity_message(False)
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_FORWARD)

        # Test post-state
        self.assertEqual(self.node._prox_sensor, False)
        self.assertEqual(self.node._current_heading, right_90)
        self.assertEqual(self.node._heading_goal, right_90)
        self.assertEqual(self.node._obstacle_forward, None)
        self.assertEqual(self.node._obstacle_right, None)
        self.assertEqual(self.node._obstacle_left, None)

    def test_fwd_right_obstacle_flow(self):
        right_90 = (2 * math.pi) - DEG_90
        left_90 = DEG_90

        print("Set initial position, with obstacle in front")
        self.node._current_heading = 0.0
        self._send_proximity_message(True)
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)

        print("Detect obstacle, and decide to turn right")
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_TURN)
        self.assertEqual(self.node._heading_goal, right_90)

        print("Complete turn, and detect right obstacle. Decide turn left.")
        self.node._current_heading = right_90
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)
        self._send_proximity_message(True)
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_TURN)
        self.assertEqual(self.node._heading_goal, left_90)

        print("Complete turn to left side, and detect clear. Decide to move forward")
        self.node._current_heading = left_90
        self._send_proximity_message(False)
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_FORWARD)
        self.assertEqual(self.node._heading_goal, left_90)

        # Test post-state
        self.assertEqual(self.node._prox_sensor, False)
        self.assertEqual(self.node._current_heading, left_90)
        self.assertEqual(self.node._heading_goal, left_90)
        self.assertEqual(self.node._obstacle_forward, None)
        self.assertEqual(self.node._obstacle_right, None)
        self.assertEqual(self.node._obstacle_left, None)

    def test_fwd_right_left_obstacle_flow(self):
        left_90 = DEG_90
        reverse_180 = DEG_90 * 2

        print("Set initial position, facing left, with obstacle in front, right, and left")
        self.node._current_heading = left_90
        self._send_proximity_message(True)
        self.node._obstacle_forward = True
        self.node._obstacle_right = True
        self.node._obstacle_left = None  # Haven't detected it yet
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)

        print("Detect obstacle on left, choose to turn to reverse out (driving forward)")
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_TURN)
        self.assertEqual(self.node._heading_goal, reverse_180)

        print("Complete turn")
        self.node._current_heading = reverse_180
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_OBSTACLE_PLAN)

        print("Detect forward clear, and drive forward")
        self._send_proximity_message(False)
        self.node._decide()
        self._print_status()
        self.assertEqual(self.node._mode, MODE_FORWARD)
        self.assertEqual(self.node._heading_goal, reverse_180)

        # Test post-state
        self.assertEqual(self.node._prox_sensor, False)
        self.assertEqual(self.node._current_heading, reverse_180)
        self.assertEqual(self.node._heading_goal, reverse_180)
        self.assertEqual(self.node._obstacle_forward, None)
        self.assertEqual(self.node._obstacle_right, None)
        self.assertEqual(self.node._obstacle_left, None)

    def _send_proximity_message(self, isblocked):
        msg = Proximity()
        msg.sensors = [isblocked]
        self.node.prox_callback(msg)
        print("  Proximity sensor:", self.node._prox_sensor)

    def _print_status(self):
        print("  Current Heading:", self.node._current_heading)
        print("  Mode:", self.node._mode)
        print("  Heading goal:", self.node._heading_goal)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestPilotNode)
