"""使用数值法计算 RX150 前三个关节的逆向运动学。"""

from math import atan2, hypot, pi

import numpy as np


# 实验讲义给出的机械臂尺寸，单位均为米。
LINK1_Z = 0.065
LINK2_Z = 0.039
LINK3_X = 0.050
LINK3_Z = 0.150
LINK4_X = 0.150


def _wrap_to_pi(angles):
    """把一个或一组角度转换到 [-pi, pi) 范围。"""

    return (angles + pi) % (2.0 * pi) - pi


def _forward_kinematics(angles):
    """根据 [joint1, joint2, joint3] 计算 joint4 的正向运动学位置。"""

    joint1, joint2, joint3 = angles
    combined = joint2 + joint3

    # 先在 joint2、joint3 所在的竖直平面中计算水平距离 radial。
    # 肩到肘的零位向量是 (LINK3_X, LINK3_Z)，旋转 joint2 后得到前两项；
    # 肘到 joint4 的方向角是 joint2 + joint3，因此得到最后一项。
    radial = (
        LINK3_X * np.cos(joint2)
        - LINK3_Z * np.sin(joint2)
        + LINK4_X * np.cos(combined)
    )
    # joint4 的高度 = 肩关节高度 + 两根连杆在 z 方向上的分量。
    vertical = (
        LINK1_Z
        + LINK2_Z
        + LINK3_X * np.sin(joint2)
        + LINK3_Z * np.cos(joint2)
        + LINK4_X * np.sin(combined)
    )

    # joint1 把竖直平面绕 z 轴旋转，因此 x = radial*cos(q1)，
    # y = radial*sin(q1)。
    return np.array(
        [radial * np.cos(joint1), radial * np.sin(joint1), vertical],
        dtype=float,
    )


def _jacobian(angles):
    """计算 3x3 位置雅可比矩阵 J = d(x, y, z)/d(q1, q2, q3)。"""

    joint1, joint2, joint3 = angles
    combined = joint2 + joint3

    radial = (
        LINK3_X * np.cos(joint2)
        - LINK3_Z * np.sin(joint2)
        + LINK4_X * np.cos(combined)
    )
    # 分别计算 radial 和 vertical 对 joint2、joint3 的偏导数。
    # 它们与 joint1 的 sin/cos 项组合后构成完整的三维雅可比矩阵。
    radial_q2 = (
        -LINK3_X * np.sin(joint2)
        - LINK3_Z * np.cos(joint2)
        - LINK4_X * np.sin(combined)
    )
    radial_q3 = -LINK4_X * np.sin(combined)
    vertical_q2 = radial
    vertical_q3 = LINK4_X * np.cos(combined)

    cos_q1 = np.cos(joint1)
    sin_q1 = np.sin(joint1)
    return np.array(
        [
            [-radial * sin_q1, radial_q2 * cos_q1, radial_q3 * cos_q1],
            [radial * cos_q1, radial_q2 * sin_q1, radial_q3 * sin_q1],
            [0.0, vertical_q2, vertical_q3],
        ],
        dtype=float,
    )


def _solve_from_seed(target, seed, maximum_iterations=300):
    """从一组初始关节角出发，使用阻尼最小二乘法迭代求解。"""

    angles = np.asarray(seed, dtype=float).copy()
    damping = 1.0e-3
    best_angles = angles.copy()
    best_error = float("inf")

    for _ in range(maximum_iterations):
        # 第 1 步：用正向运动学计算当前位置，并得到位置误差 e = target-F(q)。
        error_vector = target - _forward_kinematics(angles)
        error_norm = float(np.linalg.norm(error_vector))
        if error_norm < best_error:
            best_error = error_norm
            best_angles = angles.copy()
        if error_norm < 1.0e-9:
            break

        # 第 2 步：计算当前位置的雅可比矩阵。
        jacobian = _jacobian(angles)

        # 第 3 步：阻尼最小二乘更新：
        # delta_q = J^T (J J^T + lambda^2 I)^(-1) e。
        # 阻尼项能避免 J 在机械臂完全伸直等奇异姿态附近无法求逆。
        regularized = jacobian @ jacobian.T + damping * damping * np.eye(3)
        step = jacobian.T @ np.linalg.solve(regularized, error_vector)

        # 第 4 步：限制单次关节角变化不超过 0.35 rad，避免一步跨得过大。
        step_norm = float(np.linalg.norm(step))
        if step_norm > 0.35:
            step *= 0.35 / step_norm

        # 第 5 步：回溯搜索更新比例。只有新位置误差变小时才接受本次更新。
        improved = False
        scale = 1.0
        for _ in range(10):
            candidate = _wrap_to_pi(angles + scale * step)
            candidate_error = float(
                np.linalg.norm(target - _forward_kinematics(candidate))
            )
            if candidate_error < error_norm:
                angles = candidate
                damping = max(1.0e-6, damping * 0.7)
                improved = True
                break
            scale *= 0.5

        if not improved:
            damping = min(1.0, damping * 10.0)

    return best_angles, best_error


def inverse_kinematics(position):
    """根据 joint4 的目标位置返回 [waist, shoulder, elbow]。

    使用多组固定初值分别执行阻尼最小二乘迭代，然后选择位置误差最小的结果。
    多初值可以减少局部不收敛的问题，同时保证每次运行结果可重复。

    当输入格式错误、目标不可达或迭代不收敛时抛出 ValueError。
    """

    try:
        target = np.asarray(position, dtype=float)
    except (TypeError, ValueError):
        raise ValueError("position must contain exactly three numeric values") from None

    if target.shape != (3,):
        raise ValueError("position must contain exactly three numeric values")
    if not np.all(np.isfinite(target)):
        raise ValueError("position values must be finite")

    # 在开始迭代前先检查目标是否位于二连杆的理论工作空间内。
    radial = hypot(float(target[0]), float(target[1]))
    vertical = float(target[2]) - LINK1_Z - LINK2_Z
    distance = hypot(radial, vertical)
    first_length = hypot(LINK3_X, LINK3_Z)
    minimum_reach = abs(first_length - LINK4_X)
    maximum_reach = first_length + LINK4_X
    if distance < minimum_reach - 1.0e-9 or distance > maximum_reach + 1.0e-9:
        raise ValueError("target position is outside the RX150 workspace")

    # joint1 的初值直接使用目标点的方位角；joint2、joint3 使用多组初值。
    waist_guess = atan2(float(target[1]), float(target[0])) if radial else 0.0
    seeds = (
        (waist_guess, 0.0, 0.0),
        (waist_guess, pi / 4.0, -pi / 2.0),
        (waist_guess, pi / 2.0, -pi / 2.0),
        (waist_guess, -pi / 2.0, pi / 2.0),
        (waist_guess, 0.0, pi / 2.0),
        (waist_guess, 0.0, -pi / 2.0),
    )

    # 分别迭代求解，并选择正向回代位置误差最小的一组关节角。
    solutions = [_solve_from_seed(target, seed) for seed in seeds]
    angles, error = min(solutions, key=lambda result: result[1])
    if error > 1.0e-6:
        raise ValueError(f"numerical IK did not converge (position error {error:.3g} m)")

    # 内部 FK 模型将 shoulder/elbow 视为绕 -Y 旋转，而 URDF 中为绕 +Y 旋转，
    # 因此需要对 joint2 与 joint3 取反。
    return [
        float(angles[0]),
        float(_wrap_to_pi(-angles[1])),
        float(_wrap_to_pi(-angles[2])),
    ]
