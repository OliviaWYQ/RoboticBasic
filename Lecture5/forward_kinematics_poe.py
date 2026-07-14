"""RX150 正向运动学：指数积（Product of Exponentials, PoE）方法。"""

import numpy as np


def _skew(vector):
    """把三维向量转换为用于叉乘的反对称矩阵。"""
    x, y, z = vector
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def _screw_exponential(screw_axis, theta):
    """计算旋转关节的矩阵指数 exp([S] * theta)。"""
    omega = screw_axis[:3]
    velocity = screw_axis[3:]
    omega_hat = _skew(omega)
    omega_hat_squared = omega_hat @ omega_hat

    rotation = (
        np.eye(3)
        + np.sin(theta) * omega_hat
        + (1.0 - np.cos(theta)) * omega_hat_squared
    )
    translation_matrix = (
        theta * np.eye(3)
        + (1.0 - np.cos(theta)) * omega_hat
        + (theta - np.sin(theta)) * omega_hat_squared
    )

    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation_matrix @ velocity
    return transform


def forward_kinematics(joints):
    """根据 [joint1, joint2, joint3]（弧度）返回 joint 4 的 [x, y, z]（米）。"""
    joint_angles = np.asarray(joints, dtype=float).reshape(-1)
    if joint_angles.size != 3:
        raise ValueError("joints 必须包含三个关节角 [joint1, joint2, joint3]")

    # PDF 原理图中的尺寸，单位由毫米换算为米。
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    shoulder_height = link1z + link2z

    # M：三个关节角都为 0 时，joint 4 相对于 base_link 的位姿。
    home_configuration = np.eye(4)
    home_configuration[:3, 3] = [
        link3x + link4x,
        0.0,
        shoulder_height + link3z,
    ]

    # 每一行是一个 space screw axis S = [omega, velocity]。
    # joint1 绕 z 轴；joint2、joint3 绕 y 轴。
    screw_axes = np.array([
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, -shoulder_height, 0.0, 0.0],
        [
            0.0,
            1.0,
            0.0,
            -(shoulder_height + link3z),
            0.0,
            link3x,
        ],
    ])

    # PoE 公式：T = exp([S1]q1) exp([S2]q2) exp([S3]q3) M。
    transform = np.eye(4)
    for screw_axis, angle in zip(screw_axes, joint_angles):
        transform = transform @ _screw_exponential(screw_axis, angle)
    transform = transform @ home_configuration

    return transform[:3, 3].tolist()
