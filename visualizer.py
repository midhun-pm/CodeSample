import numpy as np
import config
from matplotlib import pyplot as plt
import util

def draw_car(x, y, heading, label=None):
    '''
    Plots a car at a position

    :param x: X coordinate in m
    :param y: Y coordinate in m
    :param heading: Angle in radians
    :param label: An optional label for the mid point of rear axel
    :return:
    '''
    plt.axis('equal')
    back_point = np.array([0, 0, 1]).reshape(3, 1)
    front_point = np.array([config.DISTANCE_FROM_FRONT_WHEEL_BACK_AXIS, 0, 1]).reshape(3, 1)
    left_wheel_point = np.array([0, config.DISTANCE_BETWEEN_REAR_WHEELS / 2, 1]).reshape(3, 1)
    right_wheel_point = np.array([0, -config.DISTANCE_BETWEEN_REAR_WHEELS / 2, 1]).reshape(3, 1)

    rot_trans = util.calc_rotation_translation_matrix_homogeneous(x, y, heading)

    back_point_final = rot_trans.dot(back_point)
    back_point_final_x, back_point_final_y, _ = np.ravel(back_point_final)

    front_point_final = rot_trans.dot(front_point)
    left_wheel_point_final = rot_trans.dot(left_wheel_point)
    right_wheel_point_final = rot_trans.dot(right_wheel_point)

    front_point_final_x, front_point_final_y, _ = np.ravel(front_point_final)

    left_wheel_point_final_x, left_wheel_point_final_y, _ = np.ravel(left_wheel_point_final)
    right_wheel_point_final_x, right_wheel_point_final_y, _ = np.ravel(right_wheel_point_final)

    plt.scatter(back_point_final_x, back_point_final_y, color='white', facecolors='none')
    plt.scatter(front_point_final_x, front_point_final_y, color='red')
    if label is not None:
        plt.text(back_point_final_x, back_point_final_y, label, ha='center', va='center')
    plt.plot([back_point_final_x, front_point_final_x], [back_point_final_y, front_point_final_y])

    plt.plot([left_wheel_point_final_x, right_wheel_point_final_x],
             [left_wheel_point_final_y, right_wheel_point_final_y], color='black', linestyle=':')


def plot_points(x_array, y_array):
    '''
    Plots a path connecting points
    :param x_array: Array of x co-ordinates
    :param y_array: Array of y co-ordinates
    :return: None
    '''
    plt.plot(x_array, y_array)
    plt.scatter(x_array, y_array)


def show():
    plt.show()
