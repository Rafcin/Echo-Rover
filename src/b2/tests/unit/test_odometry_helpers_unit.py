import unittest
import math

from b2_logic.odometry_helpers import normalize_theta, calc_world_frame_pose, calc_steering_angle

PKG = 'b2'
NAME = 'b2_odom_helpers_unittest'

pi = math.pi
twopi = pi * 2


class TestOdometryHelpers(unittest.TestCase):

    def setUp(self):
        print()

    def test_normalize_theta(self):
        # (input_theta, exp_normal_theta)
        tests = [
            # Positive & Negatives
            (pi, pi),               # 180 = 180
            (-pi, pi),              # -180 = 180
            (twopi, 0.0),           # 360 = 0
            (-twopi, 0.0),          # -360 = 0
            (0, 0),                 # 0 = 0
            (pi/2 - pi, pi * 1.5),  # -90 = 270

            # More that 2 Pi cases
            (twopi + pi, pi),       # 3 pi = pi
            (-twopi - pi, pi),      # -3 pi = pi
            (pi + pi * 1.5, pi/2)   # 2.5 pi = 0.5 pi
        ]

        for input_theta, exp_normal_theta in tests:
            normal_theta = normalize_theta(input_theta)
            print("Input theta: {}, actual: {}, expected: {}".format(
                input_theta, normal_theta, exp_normal_theta))
            self.assertAlmostEqual(normal_theta, exp_normal_theta, 3)

    def test_calc_steering_angle(self):
        # (current_heading, target_heading, exp_steering_angle)
        tests = [
            (pi/2, pi, pi/2),       # 90 deg -> 180 deg = +90 deg
            (pi/4, -pi/4, -pi/2),   # 45 deg -> -45 deg = -90 deg
            (1.1, 0.0, -1.1),
            (twopi - 1.0, 0.0, 1.0),
            (1.0, twopi - 1.0, -2.0),
        ]

        for (current_heading, target_heading, exp_steering_angle) in tests:
            steering_angle = calc_steering_angle(current_heading, target_heading)
            print(
                "Current heading: {}, Target heading: {} ==> "
                "Steering angle actual: {}, expected: {}".format(
                    current_heading, target_heading, steering_angle, exp_steering_angle))
            self.assertAlmostEqual(
                steering_angle, exp_steering_angle, 3)

    def test_calc_world_frame_pose(self):
        """Tests the odometry_helpers.calc_world_frame_pose() function.
        Inputs: world x-y-theta velocities, and world starting coordinates, and duration
        Outputs: new world x-y-theta pose
        """
        tests = [
            # (world_x_velocity, world_y_velocity, world_angular_velocity,
            #  begin_world_x, begin_world_y, begin_world_theta, time_delta_secs),
            # ==> (new_world_x, new_world_y, new_world_theta)

            # Drive straight forward at 0.5 m/s for 1 sec from origin
            ((0.5, 0.0, 0.0,
              0.0, 0.0, 0.0, 1),
             (0.5, 0.0, 0.0)),

            # Rotate left at 1 r/s for 1 sec from origin
            ((0.0, 0.0, 1.0,
              0.0, 0.0, 0.0, 1),
             (0.0, 0.0, 1.0)),

            # Rotate left at 3 r/s for 3 sec from origin
            ((0.0, 0.0, 3.0,
              0.0, 0.0, 0.0, 3),
             (0.0, 0.0, (3 * 3) % math.pi)),

            # Rotate right at 1 r/s for 1 sec from origin
            ((0.0, 0.0, -1.0,
              0.0, 0.0, 0.0, 1),
             (0.0, 0.0, 5.28)),

            # Drive x = 1.0 m/s, y = 1.0ms/s and rotation 1.0 r/s for 1 sec
            # from the origin and 0.0 heading
            ((1.0, 1.0, 1.0,
              0.0, 0.0, 0.0, 1),
             (1.0, 1.0, 1.0)),

            # Drive x = 1.0 m/s, y = 1.0ms/s and rotation 1.0 r/s for 1 sec
            # from the location (-123, 345) heading = 4 radians
            ((1.0, 1.0, 1.0,
              -123.0, 345.0, 4.0, 1),
             (-122.0, 346.0, 5.0)),
        ]

        for inputs, expects in tests:
            (world_x_velocity, world_y_velocity, world_angular_velocity,
             begin_world_x, begin_world_y, begin_world_theta, time_delta_secs) = inputs

            exp_world_x, exp_world_y, exp_world_theta = expects

            new_world_x, new_world_y, new_world_theta = calc_world_frame_pose(
                world_x_velocity, world_y_velocity, world_angular_velocity,
                begin_world_x, begin_world_y, begin_world_theta,
                time_delta_secs
            )

            self.assertAlmostEqual(new_world_x, exp_world_x, places=2)
            self.assertAlmostEqual(new_world_y, exp_world_y, places=2)
            self.assertAlmostEqual(new_world_theta, exp_world_theta, places=2)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun(PKG, NAME, TestOdometryHelpers)
