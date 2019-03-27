# A collection of estimators

# by Midhun Pookkottil Madhusoodanan

import numpy as np
import config
import util
import visualizer
import logging


class PoseEstimator(object):
    '''
        The pose estimator.
        Maintains internal state of the pose
    '''

    def __init__(self, initial_pose, time, steering_angle, encoder_ticks, angular_velocity):
        '''
        The first call should initialize the values, primarily for the ticker.

        :param initial_pose: tuple eg: (0,0,0)
        :param time: in seconds
        :param steering_angle: in radians
        :param encoder_ticks: integer
        :param angular_velocity: rad/sec
        '''
        self.x, self.y, self.heading = initial_pose
        self.timestamp = time
        self.encoder_ticks = encoder_ticks
        self.angular_velocity = angular_velocity
        self.steering_angle = steering_angle
        self.front_wheel_circumference_meters = 2.0 * np.pi * config.FRONT_WHEEL_RADIUS_METERS

    def estimate(self, time, steering_angle, encoder_ticks, angular_velocity):
        '''
        Returns the estimated pose after the event

        :param time: in seconds
        :param steering_angle: in radians
        :param encoder_ticks: integer
        :param angular_velocity: radians / sec
        :return: estimated_pose(x(meters),y(meters),heading(?))
        '''

        # Calculate deltas in ticks and time
        delta_ticks = encoder_ticks - self.encoder_ticks

        delta_time = time - self.timestamp

        # Calculate distance travelled by the front wheel using tick deltas
        distance_travelled_front_wheel_ticks = self.calc_distance_travelled_by_front_wheel(delta_ticks)

        # TODO the relevance of the angular velocity measurement is unclear from the description.
        # Assuming that the angular velocity is calculated at around an axes that falls at the vertical & horizontal center of the platform
        # Perhaps it indicates the skid of the wheels ??
        #
        # Or is it a second sensor for additional accuracy information using sensor fusion with a Kalman filter perhaps?
        # Since, I was unclear about the dynamics here, I choose to rely only on the ticks for this code sample.
        skid_distance = 0

        if self.angular_velocity is not None and delta_time != 0:
            skid_distance = self.calc_distance_travelled_by_front_wheel_skid(steering_angle,
                                                                             initial_angular_velocity=self.angular_velocity,
                                                                             final_angular_velocity=angular_velocity,
                                                                             delta_time=delta_time)

        # TODO , I have a feeling that if my assumption about the angular velocity is correct, the skid need to be incorporated
        # into the calculations here. However, I'm leaving it out for the time being.
        total_dist = distance_travelled_front_wheel_ticks

        if total_dist == 0:
            return (self.x, self.y, self.heading)

        self.x, self.y, self.heading = self.update_position(steering_angle, total_dist)

        self.steering_angle = steering_angle
        self.angular_velocity = angular_velocity
        self.timestamp = time
        self.encoder_ticks = encoder_ticks

        return (self.x, self.y, self.heading)

    def calc_outer_turn_radius(self, steering_angle_radians):
        '''

        Three wheeled vehicle will follow a bicycle turning model.
        In which front and mid point of rear axel will follow a circle with the same center
        sin(angle_subtended_at_center) = length_of_vehicle / outer_turn_radius

        :param steering_angle_radians:
        :return: turn radius in meters
        '''

        if steering_angle_radians != 0:
            outer_turn_radius = np.abs(config.DISTANCE_FROM_FRONT_WHEEL_BACK_AXIS / np.sin(steering_angle_radians))
        else:
            outer_turn_radius = float('inf')
        logging.info("Outer turn radius: " + str(outer_turn_radius))
        return outer_turn_radius

    def calc_inner_turn_radius(self, steering_angle_radians):
        '''

        Three wheeled vehicle will follow a bicycle turning model.
        In which front and mid point of rear axel will follow a circle with the same center

        tan(angle_subtended_at_center) = length_of_vehicle / inner_turn_radius

        Here inner_turn_radius is the distance from the turn center to the mid-point of the rear axel.

        :param steering_angle_radians:
        :return: turn radius in meters

        '''

        inner_turn_radius = np.abs(config.DISTANCE_FROM_FRONT_WHEEL_BACK_AXIS / np.tan(steering_angle_radians))
        logging.info("Inner turn radius: " + str(inner_turn_radius))
        return inner_turn_radius

    def calc_angle_swept_center(self, distance_travelled_by_front_wheel_meters, outer_turn_radius_meters):
        '''
        2*pi*outer_turn_radius * theta / 2*pi = distance_travelled_by_front_wheel_meters , solve for theta

        :param distance_travelled_by_front_wheel_meters: distance travelled by front wheel meters
        :param outer_turn_radius_meters: The radius of turn of the outer wheel

        :return:
        '''
        angle_subtended_by_arc_of_turn_at_center = distance_travelled_by_front_wheel_meters / outer_turn_radius_meters
        logging.info("Angle Subtended By Arc of Turn At Center :" + str(angle_subtended_by_arc_of_turn_at_center))
        return angle_subtended_by_arc_of_turn_at_center

    def calc_distance_travelled_by_front_wheel(self, tick_count_int):
        '''
        Calculates the distance travelled which is a fraction (ticks/resolution) of the circumference of the wheel

        :param tick_count_int:
        :return: distance travelled by front wheel in meters ( can be a negative value if whell rotated backwards )
        '''
        distance = self.front_wheel_circumference_meters / config.ENCODER_RESOLUTION_FRONT_WHEEL * tick_count_int
        logging.info("Distance Travelled Front Wheel: " + str(distance))
        return distance

    def calc_distance_travelled_by_front_wheel_skid(self, steering_angle_radians, initial_angular_velocity,
                                                    final_angular_velocity, delta_time):
        '''
        Calculates the distance travelled as inferred from the change in angular velocity.
        :param steering_angle_radians:
        :param initial_angular_velocity: in rad/sec
        :param final_angular_velocity: in rad/sec
        :param delta_time: in sec
        :return:
        '''
        if steering_angle_radians == 0:
            return 0

        angular_accleration = (final_angular_velocity - initial_angular_velocity) / delta_time
        angular_displacement = initial_angular_velocity * delta_time + 0.5 * angular_accleration * (delta_time ** 2)
        platform_length = config.DISTANCE_FROM_FRONT_WHEEL_BACK_AXIS
        distance_travelled = platform_length / 2 * angular_displacement  # 2 * np.pi * outer_turn_radius * angular_displacement / 2*np.pi
        return distance_travelled

    def calc_delta_translation_heading(self, steering_angle_radians, distance_travelled_by_front_wheel_meters):

        outer_turn_radius = self.calc_outer_turn_radius(steering_angle_radians)

        angle_subtended_by_arc_of_turn_at_center = self.calc_angle_swept_center(
            distance_travelled_by_front_wheel_meters, outer_turn_radius)

        inner_turn_radius = self.calc_inner_turn_radius(steering_angle_radians)

        x_wrt_center, y_wrt_center = util.polar_to_cartisian(inner_turn_radius,
                                                             angle_subtended_by_arc_of_turn_at_center)

        translation_wrt_turn_center = util.calc_rotation_translation_matrix_homogeneous(x_wrt_center, y_wrt_center, 0)

        turn_center_to_mid_axel = util.calc_rotation_translation_matrix_homogeneous(inner_turn_radius, 0, 0)

        delta_translation_wrt_axel_center = turn_center_to_mid_axel.dot(translation_wrt_turn_center)

        delta_heading = angle_subtended_by_arc_of_turn_at_center

        return delta_translation_wrt_axel_center, delta_heading

    def update_position(self, steering_angle_radians, distance_travelled_by_front_wheel_meters):

        # Calculate the transformation matrix of the current position of the car
        original_rot_trans_mat = util.calc_rotation_translation_matrix_homogeneous(self.x, self.y, self.heading)

        # If executing a turn
        if steering_angle_radians != 0:
            delta_trans_mat, delta_heading = self.calc_delta_translation_heading(steering_angle_radians,
                                                                                 distance_travelled_by_front_wheel_meters)
        else:
            # If no turn, we're moving forward
            delta_trans_mat = util.calc_rotation_translation_matrix_homogeneous(
                distance_travelled_by_front_wheel_meters, 0, 0)

            # No change in heading
            delta_heading = 0

        # New location (of back of cart) wrt previous location (of back of cart)
        new_tranformation_wrt_cart = delta_trans_mat.dot(np.array([[0], [0], [1]]))

        # Final location wrt world frame, the previous location of the cart
        final_point = original_rot_trans_mat.dot(new_tranformation_wrt_cart)

        x, y, _ = np.ravel(final_point)
        heading = self.heading + delta_heading

        return x, y, heading
