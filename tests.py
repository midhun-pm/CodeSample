import unittest
from estimator import PoseEstimator
import numpy as np
import config


class PoseEstimatorTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_outer_radius(self):
        pose_estimator = PoseEstimator((0, 0, 0), 0, 0, 0, 0)

        steering_angle_radians = np.pi / 2
        self.assertEqual(config.DISTANCE_FROM_FRONT_WHEEL_BACK_AXIS,
                         pose_estimator.calc_outer_turn_radius(steering_angle_radians))

        steering_angle_radians = np.pi / 4
        self.assertEqual(config.DISTANCE_FROM_FRONT_WHEEL_BACK_AXIS / np.sin(steering_angle_radians),
                         pose_estimator.calc_outer_turn_radius(steering_angle_radians))

        steering_angle_radians = 0
        self.assertEqual(np.inf, pose_estimator.calc_outer_turn_radius(steering_angle_radians))

    def test_straight_location_estimation(self):
        # Testing the position for straight line motion
        pose_estimator = PoseEstimator((0, 0, 0), 0, 0, 0, 0)
        pose_estimator.estimate(0, 0, 0, 0)
        self.assertEqual((2 * np.pi * config.FRONT_WHEEL_RADIUS_METERS, 0, 0), pose_estimator.estimate(1, 0, 512, 10))


    def test_turn_location_position(self):
        # Human assisted test
        # Ensure that the car returns to the same location if we drive around in circles.

        import visualizer
        pose_estimator = PoseEstimator((0, 0, 0), 0, 0, 0, 0)
        d = 4
        steering_angle_radians = np.pi / d
        outer_turn_radius_meters = pose_estimator.calc_outer_turn_radius(steering_angle_radians)
        front_wheel_turn_circumference = 2 * np.pi * config.FRONT_WHEEL_RADIUS_METERS
        turn_circle_circumference = 2 * np.pi * outer_turn_radius_meters
        ticks_required =   config.ENCODER_RESOLUTION_FRONT_WHEEL * turn_circle_circumference / front_wheel_turn_circumference
        result_loc = []
        ticks = 0
        for i in range(2*d):
            result_loc.append(pose_estimator.estimate(time=i, steering_angle=steering_angle_radians, encoder_ticks=ticks, angular_velocity=0))
            ticks = ticks + ticks_required / (2*d)
        for loc in result_loc:
            visualizer.draw_car(loc[0], loc[1], loc[2])
        visualizer.show()
