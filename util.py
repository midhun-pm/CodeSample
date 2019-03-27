import numpy as np


def euclidean_distance(pointA, pointB):
    '''
    Calculates the euclidean distance between 2 points
    :param pointA: [x, y] in meters
    :param pointB: [x, y] in meters
    :return: distance
    '''
    return np.linalg.norm(pointA-pointB)


def calc_rotation_translation_matrix_homogeneous(x, y, heading_radians):
    '''
    returns a 3 x 3 matrix that corresponds to the transformation
    which involves a rotation *followed by* a translation

    :param x: x coordinate
    :param y: y coordinate
    :param heading_radians: rotation in radians
    :return:
    '''
    rot_trans_matrix = np.mat(
        [
            [np.cos(heading_radians), -1.0 * np.sin(heading_radians), x],
            [np.sin(heading_radians), np.cos(heading_radians), y],
            [0.0, 0.0, 1],

        ])
    return rot_trans_matrix

def read_csv(path_absolute):
    '''
    :param path_absolute: Absolute path to csv file
    :return: ndarray

    Assuming that the data is formatted correctly and there are no missing values,
    loadtext is faster than genfromtext
    '''
    return np.loadtxt(open(path_absolute, "rb"), delimiter=",", skiprows=1)


def polar_to_cartisian(R_meters, theta_radians):
    '''
    Convert from polar to cartisian co-ordinate system.
    :param R: radius in meters
    :param theta: in radians
    :return: x, y  The x and y coordinates in meters
    '''
    x = R_meters * np.cos(theta_radians)
    y = R_meters * np.sin(theta_radians)
    return x, y