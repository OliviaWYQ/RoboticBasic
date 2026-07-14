"""RX150 正向运动学：标准 D-H 参数方法。"""

from math import pi

import numpy as np


def _dh_transform(theta, d, a, alpha):
    """根据一行标准 D-H 参数生成 4x4 齐次变换矩阵。"""
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    return np.array([
        [
            cos_theta,
            -sin_theta * cos_alpha,
            sin_theta * sin_alpha,
            a * cos_theta,
        ],
        [
            sin_theta,
            cos_theta * cos_alpha,
            -cos_theta * sin_alpha,
            a * sin_theta,
        ],
        [0.0, sin_alpha, cos_alpha, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(joints):
    """根据 [joint1, joint2, joint3]（弧度）返回 joint 4 的 [x, y, z]（米）。"""
    joint_angles = np.asarray(joints, dtype=float).reshape(-1)
    if joint_angles.size != 3:
        raise ValueError("joints 必须包含三个关节角 [joint1, joint2, joint3]")
    joint1, joint2, joint3 = joint_angles

    # PDF 原理图中的尺寸，单位由毫米换算为米。
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150

    shoulder_height = link1z + link2z

    # link3 同时有 x、z 两个方向的偏移，因此将其换算成长度和固定夹角。
    link3_length = np.hypot(link3x, link3z)
    beta = np.arctan2(link3z, link3x)

    # 标准 D-H 参数：T = T01 @ T12 @ T23。
    transform_01 = _dh_transform(joint1, shoulder_height, 0.0, -pi / 2.0)
    transform_12 = _dh_transform(joint2 - beta, 0.0, link3_length, 0.0)
    transform_23 = _dh_transform(joint3 + beta, 0.0, link4x, 0.0)

    transform_03 = transform_01 @ transform_12 @ transform_23
    return transform_03[:3, 3].tolist()
