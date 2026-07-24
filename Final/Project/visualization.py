#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

CELL_SIZE = 0.5
GRID_ROWS = 12
GRID_COLS = 7

GRID_MAP = (
    "#######",
    "#.....#",
    "#.....#",
    "#.....#",
    "####..#",
    "#.....#",
    "#.....#",
    "#..####",
    "#.....#",
    "#.....#",
    "#.....#",
    "#######",
)

START_REGION = {(3, 9), (4, 9), (5, 9), (3, 10), (4, 10), (5, 10)}
BUFFER_CELL = (2, 2)
GOAL_CELL = (2, 1)
DEFAULT_START = (5, 10)


def cell_to_odom(cell, start_cell=DEFAULT_START):
    """将网格坐标 (col, row) 转为里程计坐标（和 turtlebot.py 一致）。"""
    dc = cell[0] - start_cell[0]
    dr = cell[1] - start_cell[1]
    x = dc * CELL_SIZE
    y = -dr * CELL_SIZE
    return x, y


def visualization():
    _, ax = plt.subplots(figsize=(10, 14))
    ax.set_aspect('equal')

    start_cell = DEFAULT_START

    # 画障碍物（里程计坐标）
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            if GRID_MAP[row][col] == '#':
                x, y = cell_to_odom((col, row), start_cell)
                ax.add_patch(Rectangle((x - CELL_SIZE/2, y - CELL_SIZE/2),
                                       CELL_SIZE, CELL_SIZE,
                                       color='gray', ec='black', linewidth=0.5))

    # 画起点区域（绿色）
    for col, row in START_REGION:
        x, y = cell_to_odom((col, row), start_cell)
        ax.add_patch(Rectangle((x - CELL_SIZE/2, y - CELL_SIZE/2),
                               CELL_SIZE, CELL_SIZE,
                               color='lightgreen', alpha=0.4))

    # 画目标格（红色）
    gx, gy = cell_to_odom(GOAL_CELL, start_cell)
    ax.add_patch(Rectangle((gx - CELL_SIZE/2, gy - CELL_SIZE/2),
                           CELL_SIZE, CELL_SIZE,
                           color='red', alpha=0.5))
    ax.text(gx, gy, 'GOAL', ha='center', va='center',
            fontsize=8, fontweight='bold', color='white')

    # 画缓冲格（橙色）
    bx, by = cell_to_odom(BUFFER_CELL, start_cell)
    ax.add_patch(Rectangle((bx - CELL_SIZE/2, by - CELL_SIZE/2),
                           CELL_SIZE, CELL_SIZE,
                           color='orange', alpha=0.5))
    ax.text(bx, by, 'BUFFER', ha='center', va='center',
            fontsize=7, fontweight='bold')

    # 画 A* 规划路径（里程计坐标）
    from turtlebot import build_mission_path
    path_cells = build_mission_path(start_cell)
    path_xy = [cell_to_odom(cell, start_cell) for cell in path_cells]
    start_xy = cell_to_odom(start_cell, start_cell)  # (0, 0)
    all_xy = [start_xy] + path_xy
    px, py = zip(*all_xy)
    ax.plot(px, py, 'b--', linewidth=1.5, alpha=0.6, label='Planned A* path')
    ax.scatter(px, py, c='blue', s=20, alpha=0.6)
    ax.scatter(*start_xy, c='green', s=120, marker='o',
               zorder=5, label='Start', edgecolors='black')

    # 画实际轨迹
    try:
        trajectory = np.loadtxt("trajectory.csv", delimiter=',')
        if trajectory.ndim == 1:
            trajectory = trajectory.reshape(-1, 2)
        ax.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=2,
                label='Actual trajectory')
        ax.scatter(trajectory[-1, 0], trajectory[-1, 1], c='red', s=120,
                   marker='*', zorder=5, label='End', edgecolors='black')
    except OSError:
        print("trajectory.csv not found. Run turtlebot.py first.")

    ax.legend(loc='lower left')
    ax.set_title('Final Project: A* Path + Actual Trajectory')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    visualization()
