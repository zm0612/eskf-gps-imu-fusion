import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


def load_txt_data(data_path):
    try:
        return np.loadtxt(data_path)
    except FileNotFoundError as err:
        print('this is a OSError: ' + str(err))


if __name__ == "__main__":
    fuse_data_path = './fused.txt'
    gps_data_path = './gps_measurement.txt'
    gt_data_path = './gt.txt'

    fuse_data = load_txt_data(fuse_data_path)
    gps_data = load_txt_data(gps_data_path)
    gt_data = load_txt_data(gt_data_path)

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_title('compare path')
    ax.plot3D(fuse_data[:, 1], fuse_data[:, 2], fuse_data[:, 3], color='r', label='fuse_gps_imu')
    ax.plot3D(gps_data[:, 1], gps_data[:, 2], gps_data[:, 3], color='g', alpha=0.5, label='gps')
    ax.plot3D(gt_data[:, 1], gt_data[:, 2], gt_data[:, 3], color='b', label='ground_truth')
    plt.legend(loc='best')
    plt.show()
