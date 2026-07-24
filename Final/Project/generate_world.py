#!/usr/bin/env python3
"""生成带障碍墙的 Gazebo world 文件。"""

import os

CELL = 0.5          # 网格大小
HEIGHT = 0.25       # 墙高
GRID_ROWS = 12
GRID_COLS = 7
START = (5, 10)     # 起点网格

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

def cell_to_odom(col, row):
    x = (col - START[0]) * CELL
    y = -(row - START[1]) * CELL
    return x, y

# 生成所有障碍方块
wall_models = []
wall_id = 0
for row in range(GRID_ROWS):
    for col in range(GRID_COLS):
        if GRID_MAP[row][col] == '#':
            x, y = cell_to_odom(col, row)
            wall_models.append(f"""
    <model name="wall_{wall_id}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {HEIGHT/2:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{CELL:.3f} {CELL:.3f} {HEIGHT:.3f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>{CELL:.3f} {CELL:.3f} {HEIGHT:.3f}</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>""")
            wall_id += 1

world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="grid_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- 地面 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    {''.join(wall_models)}
  </world>
</sdf>
"""

output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grid_world.world")
with open(output_path, 'w') as f:
    f.write(world_content)

print(f"Generated {wall_id} walls -> {output_path}")
