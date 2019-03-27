# Driver code that reads values and calls the pose estimator for estimates

# by Midhun Pookkottil Madhusoodanan

import numpy as np
from estimator import PoseEstimator
import util
import visualizer
import config
import logging

np.set_printoptions(threshold=np.nan)
np.set_printoptions(suppress=True)

# Set the logging level
logging.basicConfig(level=logging.ERROR)


def main():
    # Reads from the data file and runs estimate for each row
    # Then plots the trajectory
    data_array = util.read_csv(config.DATASET_ABSOLUTE_PATH)

    row = data_array[0]
    time, encoder, angular_velocity, steering_angle = np.ravel(row)
    resulting_pos_heading = []
    pose_estimator = PoseEstimator((0, 0, 0), time, encoder, angular_velocity, steering_angle)
    i=1
    while i < len(data_array):
        row = data_array[i]
        time, encoder, angular_velocity, steering_angle = np.ravel(row)
        x, y, heading = pose_estimator.estimate(time=time, steering_angle=steering_angle, encoder_ticks=encoder,
                                                angular_velocity=angular_velocity)
        resulting_pos_heading.append([x, y, heading])
        i = i + 1
    visualizer.plot_points(np.asarray(resulting_pos_heading)[:, 0], np.asarray(resulting_pos_heading)[:, 1])
    visualizer.show()


if __name__ == "__main__":
    main()
