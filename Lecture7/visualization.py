#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

def visualization():
    # load csv file and plot actual trajectory
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    # 规划的路标点（和 trajectory_generation.py 里的 waypoints 一致）
    waypoints = np.array([
        [0.0, 0.0], [0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
        [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
        [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]
    ])

    # 实际轨迹
    trajectory = np.loadtxt("trajectory.csv", delimiter=',')

    # 画图
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro--', linewidth=1,
             markersize=4, label='Planned waypoints')
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2,
             label='Actual trajectory')

    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend()
    plt.title('Trajectory Tracking')
    plt.xlim(-0.2, 2.0)
    plt.ylim(-0.8, 0.8)
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    visualization()
