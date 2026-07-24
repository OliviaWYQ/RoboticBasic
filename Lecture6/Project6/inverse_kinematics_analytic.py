"""使用解析法计算 RX150 前三个关节的逆向运动学。"""

from math import acos, atan2, cos, hypot, isfinite, pi, sin


# 实验讲义给出的机械臂尺寸，单位均为米。
# LINK1_Z + LINK2_Z 是肩关节相对于 base_link 的高度。
# (LINK3_X, LINK3_Z) 是 joint2 = 0 时从肩关节指向肘关节的向量。
# LINK4_X 是肘关节到 joint4 的距离。
LINK1_Z = 0.065
LINK2_Z = 0.039
LINK3_X = 0.050
LINK3_Z = 0.150
LINK4_X = 0.150


def _wrap_to_pi(angle):
    """把角度转换到 [-pi, pi) 范围，方便直接发送给关节控制器。"""

    return (angle + pi) % (2.0 * pi) - pi


def inverse_kinematics(position):
    """根据 joint4 的目标位置返回 [waist, shoulder, elbow]。

    position 是 base_link 坐标系中的 [x, y, z]，单位为米。一个目标位置通常
    存在“肘向上”和“肘向下”两组解；这里选择与讲义示例相符的肘向下解。

    当输入格式错误或目标超出机械臂工作空间时抛出 ValueError。
    """

    try:
        x, y, z = (float(value) for value in position)
    except (TypeError, ValueError):
        raise ValueError("position must contain exactly three numeric values") from None

    if not all(isfinite(value) for value in (x, y, z)):
        raise ValueError("position values must be finite")

    # 第 1 步：计算腰关节角。
    # joint1 只负责绕 z 轴旋转，因此目标点在 x-y 平面内的方位角就是 joint1。
    joint1 = atan2(y, x) if x != 0.0 or y != 0.0 else 0.0

    # 第 2 步：把三维问题转换为 r-z 竖直平面内的二连杆问题。
    # radial 是目标点到 z 轴的水平距离；vertical 是目标相对肩关节的高度。
    radial = hypot(x, y)
    vertical = z - LINK1_Z - LINK2_Z

    # 第 3 步：把肩到肘的倾斜结构等效为长度 first_length 的第一根连杆。
    # first_offset 是该连杆在 joint2 = 0 时相对水平线已有的固定偏角 alpha。
    first_length = hypot(LINK3_X, LINK3_Z)
    first_offset = atan2(LINK3_Z, LINK3_X)
    second_length = LINK4_X
    target_distance = hypot(radial, vertical)

    # 二连杆能够到达的距离范围是 |a-b| <= d <= a+b。
    minimum_reach = abs(first_length - second_length)
    maximum_reach = first_length + second_length
    tolerance = 1.0e-9
    if target_distance < minimum_reach - tolerance or target_distance > maximum_reach + tolerance:
        raise ValueError("target position is outside the RX150 workspace")

    # 第 4 步：使用余弦定理求两根等效连杆之间的夹角 delta：
    # cos(delta) = (d^2 - a^2 - b^2) / (2ab)。
    cos_delta = (
        target_distance * target_distance
        - first_length * first_length
        - second_length * second_length
    ) / (2.0 * first_length * second_length)
    cos_delta = max(-1.0, min(1.0, cos_delta))

    # arccos 有正负两个分支。负号对应本实验采用的肘向下构型。
    delta = -acos(cos_delta)

    # 第 5 步：先求第一根等效连杆相对水平线的几何方向 beta。
    # atan2(vertical, radial) 是肩到目标点的方向；第二个 atan2 是三角形
    # 中目标方向与第一根连杆之间的夹角。
    first_direction = atan2(vertical, radial) - atan2(
        second_length * sin(delta),
        first_length + second_length * cos(delta),
    )
    # RX150 的 joint2 = 0 时第一根连杆已经带有 first_offset 偏角，
    # 所以需要从几何方向中减去这个固定偏角。
    joint2 = first_direction - first_offset

    # 第 6 步：由两根连杆方向之差求 joint3。
    # delta = (joint2 + joint3) - (joint2 + first_offset)
    #       = joint3 - first_offset，因此 joint3 = delta + first_offset。
    joint3 = delta + first_offset

    # 第 7 步：将 IK 内部坐标系中的关节角转换为 URDF 坐标系。
    # 内部模型将 shoulder/elbow 视为绕 -Y 旋转，而 URDF 中为绕 +Y 旋转，
    # 因此需要取反 joint2 与 joint3。返回弧度制关节角，并统一到 [-pi, pi) 范围。
    return [_wrap_to_pi(joint1), _wrap_to_pi(-joint2), _wrap_to_pi(-joint3)]
