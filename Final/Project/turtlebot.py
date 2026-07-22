#!/usr/bin/env python3
"""实验 9：使用 A* 和三次多项式轨迹完成 TurtleBot 自主导航。

程序的完整工作流程如下：

1. 用字符地图 ``GRID_MAP`` 表示 PDF 中的 7 x 12 网格环境；
2. 使用四连通 A* 算法，从绿色起点规划到橙色缓冲格；
3. 再从橙色缓冲格规划到红色目标格，保证机器人从球的下方推球；
4. 将 A* 返回的每个网格中心转换成里程计坐标系中的路标点；
5. 对相邻路标点分别生成零起止速度的三次多项式；
6. 根据 ``/odom`` 反馈进行闭环跟踪，通过 ``/cmd_vel`` 控制机器人；
7. 到达红色目标格后停止，并将实际运动轨迹保存为 CSV 文件。

网格坐标使用 ``(column, row)``，即 ``(列, 行)``：左上角是 ``(0, 0)``，
向右列号增大，向下行号增大。它与机器人使用的笛卡尔坐标不同，所以
``cell_to_odometry`` 会负责坐标转换。

在 ROS 2/Gazebo 终端中运行::

    python3 turtlebot.py

The default start cell can be changed below or overridden without editing::

    python3 turtlebot.py --ros-args -p start_col:=3 -p start_row:=9

没有 ROS 时，可以使用 ``python3 turtlebot.py --plan-only`` 单独检查路径。
"""

from __future__ import annotations

import csv
from heapq import heappop, heappush
from itertools import count
import math
import os
import sys
import time
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


Cell = Tuple[int, int]

# ---------------------------------------------------------------------------
# PDF 中的地图和用户最常修改的配置
# ---------------------------------------------------------------------------

# 每个网格边长是 0.5 m。DEFAULT_START_CELL 可以改成下方六个绿色格之一。
CELL_SIZE = 0.5
DEFAULT_START_CELL: Cell = (5, 10)
GOAL_CELL: Cell = (2, 1)       # PDF 中的红色目标格
BUFFER_CELL: Cell = (2, 2)     # 橙色缓冲格，进入目标前必须先经过它
START_REGION: Set[Cell] = {
    (3, 9), (4, 9), (5, 9),
    (3, 10), (4, 10), (5, 10),
}

# ``#`` 表示灰色墙壁或障碍物，``.`` 表示机器人可以经过的单元格。
# 下面每个字符串是一行，从上到下对应 PDF 第 2 页中的 7 列 x 12 行地图。
# 红色、橙色和绿色区域本质上仍是可通行格，因此在字符地图中也写作 ``.``。
GRID_MAP: Tuple[str, ...] = (
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

GRID_ROWS = len(GRID_MAP)
GRID_COLS = len(GRID_MAP[0])
OBSTACLES: Set[Cell] = {
    (column, row)
    for row, line in enumerate(GRID_MAP)
    for column, value in enumerate(line)
    if value == "#"
}

# 实验 8 要求只能上、左、右、下移动，不允许对角线移动。
# 把“向上”放在第一位，可以在存在多条等长最短路时得到稳定、直观的结果。
_DIRECTIONS: Tuple[Cell, ...] = ((0, -1), (-1, 0), (1, 0), (0, 1))


def _manhattan(a: Cell, b: Cell) -> int:
    """Return the admissible Manhattan heuristic for a four-connected grid."""

    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _reconstruct_path(came_from: Dict[Cell, Cell], start: Cell, goal: Cell) -> List[Cell]:
    """Return an A* path containing ``goal`` but not ``start``."""

    path: List[Cell] = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def get_path_from_A_star(
    start: Cell,
    goal: Cell,
    obstacles: Iterable[Cell],
    grid_shape: Optional[Tuple[int, int]] = None,
) -> List[Cell]:
    """Find an optimal four-connected path with A*.

    The returned list follows the Experiment 8 contract: it contains the goal
    and does not contain the start.  ``grid_shape`` is ``(rows, columns)`` and
    confines the search to a known map.  It is optional so this function keeps
    the original three-argument course API and remains independently testable.
    An empty list means either no path exists or the start already is the goal.
    """

    # 元组是可哈希对象，可以作为集合元素和字典键；列表则不可以。
    start = tuple(start)
    goal = tuple(goal)
    blocked = {tuple(cell) for cell in obstacles}

    if start in blocked or goal in blocked or start == goal:
        return []

    if grid_shape is not None:
        rows, columns = grid_shape

        def in_bounds(cell: Cell) -> bool:
            return 0 <= cell[0] < columns and 0 <= cell[1] < rows

        if not in_bounds(start) or not in_bounds(goal):
            return []
    else:
        # The original exercise API has no map dimensions.  One clear outer
        # ring is enough for any shortest detour around the supplied obstacles.
        relevant = blocked | {start, goal}
        min_x = min(cell[0] for cell in relevant) - 1
        max_x = max(cell[0] for cell in relevant) + 1
        min_y = min(cell[1] for cell in relevant) - 1
        max_y = max(cell[1] for cell in relevant) + 1

        def in_bounds(cell: Cell) -> bool:
            return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y

    # open_heap 中每一项为 (f, h, 插入顺序, 网格)。
    # f = g + h：g 是已经走过的真实代价，h 是到目标的曼哈顿距离。
    # 插入顺序只用于稳定地处理 f、h 完全相同的节点。
    serial = count()
    start_h = _manhattan(start, goal)
    open_heap: List[Tuple[int, int, int, Cell]] = [
        (start_h, start_h, next(serial), start)
    ]
    g_score: Dict[Cell, int] = {start: 0}  # 当前已知的最小起点代价
    came_from: Dict[Cell, Cell] = {}       # 用来在到达目标后反向恢复路径

    while open_heap:
        queued_f, _, _, current = heappop(open_heap)
        current_g = g_score[current]

        # A better copy of a node may have been inserted after this heap entry.
        if queued_f != current_g + _manhattan(current, goal):
            continue
        if current == goal:
            return _reconstruct_path(came_from, start, goal)

        # 扩展当前网格的四个邻居。越界点和障碍物不会加入 open 集合。
        for dx, dy in _DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            if not in_bounds(neighbor) or neighbor in blocked:
                continue

            tentative_g = current_g + 1
            if tentative_g >= g_score.get(neighbor, sys.maxsize):
                continue

            # 找到了一条到 neighbor 更短的路线，因此更新其父节点和代价。
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            h = _manhattan(neighbor, goal)
            heappush(
                open_heap,
                (tentative_g + h, h, next(serial), neighbor),
            )

    return []


def build_mission_path(start: Cell = DEFAULT_START_CELL) -> List[Cell]:
    """Plan from a green start cell through the buffer and into the goal."""

    start = tuple(start)
    if start not in START_REGION:
        raise ValueError(
            "start cell must be one of the six green cells: "
            + ", ".join(map(str, sorted(START_REGION)))
        )

    # 不能只对红色格运行一次 A*，因为一条同样短的路线可能从红色格侧面
    # 进入，从而绕开球。拆成两次规划可以强制最后一步为“橙色格 -> 红色格”。
    first_part = get_path_from_A_star(
        start, BUFFER_CELL, OBSTACLES, (GRID_ROWS, GRID_COLS)
    )
    second_part = get_path_from_A_star(
        BUFFER_CELL, GOAL_CELL, OBSTACLES, (GRID_ROWS, GRID_COLS)
    )
    if not first_part or not second_part:
        raise RuntimeError("the assignment map has no valid route to the goal")

    # Planning via BUFFER_CELL is deliberate.  It makes the final movement go
    # upward across the ball's edge, pushing the ball into the red goal area.
    return first_part + second_part


def cubic_coefficients(p_start: float, p_end: float, duration: float) -> Tuple[float, ...]:
    """Return cubic coefficients with zero velocity at both endpoints."""

    if duration <= 0.0:
        raise ValueError("duration must be positive")
    # 设 p(t) = a0 + a1*t + a2*t^2 + a3*t^3，并添加四个边界条件：
    #   p(0)=p_start, p'(0)=0, p(T)=p_end, p'(T)=0。
    # 解这四个方程后得到下面的系数。x、y 方向分别使用同一公式。
    delta = p_end - p_start
    return (
        float(p_start),
        0.0,
        3.0 * delta / (duration * duration),
        -2.0 * delta / (duration * duration * duration),
    )


def evaluate_cubic(coefficients: Sequence[float], t: float) -> Tuple[float, float]:
    """Evaluate cubic position and velocity at relative time ``t``."""

    a0, a1, a2, a3 = coefficients
    position = a0 + a1 * t + a2 * t * t + a3 * t * t * t
    velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t * t
    return position, velocity


def render_grid_path(start: Cell, path: Sequence[Cell]) -> str:
    """Return a small text visualization useful for pre-Gazebo verification."""

    canvas = [list(row) for row in GRID_MAP]
    for column, row in START_REGION:
        canvas[row][column] = "s"
    for column, row in path:
        canvas[row][column] = "*"
    canvas[BUFFER_CELL[1]][BUFFER_CELL[0]] = "B"
    canvas[GOAL_CELL[1]][GOAL_CELL[0]] = "G"
    canvas[start[1]][start[0]] = "S"
    return "\n".join("".join(row) for row in canvas)


# ROS 相关导入放在纯算法函数之后，并提供非 ROS 回退。这样即使当前电脑
# 没有安装 ROS，也仍然可以导入文件并测试 A*、多项式以及 ``--plan-only``。
try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only outside a ROS install
    rclpy = None
    Twist = None
    Odometry = None
    ExternalShutdownException = RuntimeError

    class Node:  # type: ignore[no-redef]
        pass

    ROS_AVAILABLE = False


class Turtlebot(Node):
    """负责规划、坐标转换、三次轨迹跟踪和轨迹记录的 ROS 2 节点。

    控制器使用三个主要状态：

    - ``waiting_for_odometry``：等待第一条里程计消息并建立坐标锚点；
    - ``aligning``：在网格中心原地旋转，对准下一段直线路径；
    - ``tracking``：沿三次多项式跟踪到下一个网格中心。

    每次转弯都先到达网格中心再原地旋转，因此轨迹不会在拐角处“切弯”
    进入灰色障碍格，这对只有 0.5 m 宽的通道非常重要。
    """

    def __init__(self) -> None:
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS 2 Python packages are not available")
        super().__init__("turtlebot_final_project")

        # 地图参数。map_yaw 表示“地图向右方向”相对于里程计 x 轴的夹角，
        # 单位是弧度。若 Gazebo 地图右侧正好是世界 +x，保持 0 即可。
        self.declare_parameter("start_col", DEFAULT_START_CELL[0])
        self.declare_parameter("start_row", DEFAULT_START_CELL[1])
        self.declare_parameter("cell_size", CELL_SIZE)
        self.declare_parameter("map_yaw", 0.0)

        # 控制参数使用较保守的速度，为 0.5 m 网格保留足够避障余量。
        # 这些值都能在命令行通过 ``--ros-args -p 参数名:=数值`` 覆盖。
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("cruise_speed", 0.16)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("position_gain", 1.2)
        self.declare_parameter("heading_gain", 2.5)
        self.declare_parameter("waypoint_tolerance", 0.055)
        self.declare_parameter("heading_tolerance", 0.07)
        self.declare_parameter("trajectory_file", "trajectory.csv")

        self.start_cell = (
            int(self.get_parameter("start_col").value),
            int(self.get_parameter("start_row").value),
        )
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.map_yaw = float(self.get_parameter("map_yaw").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.position_gain = float(self.get_parameter("position_gain").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.waypoint_tolerance = float(
            self.get_parameter("waypoint_tolerance").value
        )
        self.heading_tolerance = float(
            self.get_parameter("heading_tolerance").value
        )
        self.trajectory_file = str(
            self.get_parameter("trajectory_file").value
        )

        if self.control_hz <= 0.0 or self.cruise_speed <= 0.0:
            raise ValueError("control_hz and cruise_speed must be positive")
        if self.cell_size <= 0.0:
            raise ValueError("cell_size must be positive")

        # 先在整数网格上规划。此时还没有 /odom，因此暂时不能生成米制路点。
        self.path_cells = build_mission_path(self.start_cell)
        self.get_logger().info(
            "A* path (%d moves): %s"
            % (len(self.path_cells), self.path_cells)
        )

        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscription = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odometry = False
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.trajectory: List[Tuple[float, float]] = []
        self.trajectory_saved = False

        # 下列变量保存状态机和当前多项式轨迹段的状态。
        self.waypoints: List[Tuple[float, float]] = []
        self.waypoint_index = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.segment_heading = 0.0
        self.segment_duration = 0.0
        self.segment_start_time = 0.0
        self.x_coefficients: Tuple[float, ...] = (0.0,) * 4
        self.y_coefficients: Tuple[float, ...] = (0.0,) * 4
        self.state = "waiting_for_odometry"
        self.finished = False

        self.timer = self.create_timer(1.0 / self.control_hz, self.control)
        self.get_logger().info(
            "Waiting for /odom. Place the robot at grid cell %s." % (self.start_cell,)
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, message: Odometry) -> None:
        """Update planar pose and use the first sample as the start-cell anchor."""

        pose = message.pose.pose
        quaternion = pose.orientation
        # Direct quaternion-to-yaw conversion avoids a tf_transformations
        # dependency, which is not installed in every ROS 2 distribution.
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        self.yaw = math.atan2(sin_yaw, cos_yaw)
        self.x = float(pose.position.x)
        self.y = float(pose.position.y)
        self.trajectory.append((self.x, self.y))

        if not self.have_odometry:
            # 不要求用户把 Gazebo 世界原点设置在起点。本程序把第一帧 /odom
            # 位置当作 start_cell 的中心，其他格子全部相对这个锚点计算。
            self.have_odometry = True
            self.anchor_x = self.x
            self.anchor_y = self.y
            self.waypoints = [
                self.cell_to_odometry(cell) for cell in self.path_cells
            ]
            self.get_logger().info(
                "Received /odom; start anchor is (%.3f, %.3f)."
                % (self.anchor_x, self.anchor_y)
            )
            self.begin_next_segment()

    def cell_to_odometry(self, cell: Cell) -> Tuple[float, float]:
        """Map a grid-cell centre to odometry coordinates relative to start."""

        # 网格中“向下”为行号增加，而笛卡尔坐标中“向上”为 y 增加，
        # 所以 local_y 前面带负号。之后再使用二维旋转矩阵应用 map_yaw。
        delta_column = cell[0] - self.start_cell[0]
        delta_row = cell[1] - self.start_cell[1]
        local_x = delta_column * self.cell_size
        local_y = -delta_row * self.cell_size
        cosine = math.cos(self.map_yaw)
        sine = math.sin(self.map_yaw)
        return (
            self.anchor_x + cosine * local_x - sine * local_y,
            self.anchor_y + sine * local_x + cosine * local_y,
        )

    def begin_next_segment(self) -> None:
        """Select a waypoint and rotate toward it before starting its clock."""

        if self.waypoint_index >= len(self.waypoints):
            self.finish_mission()
            return

        self.target_x, self.target_y = self.waypoints[self.waypoint_index]
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        if math.hypot(dx, dy) <= self.waypoint_tolerance:
            self.waypoint_index += 1
            self.begin_next_segment()
            return

        # TurtleBot 是差速机器人，不能直接横向移动。先进入 aligning 状态
        # 原地转向，只有航向误差足够小时才启动这一段的多项式时钟。
        self.segment_heading = math.atan2(dy, dx)
        self.state = "aligning"
        self.publish_velocity(0.0, 0.0)

    def start_cubic_segment(self) -> None:
        """Build a relative-time cubic from the measured pose to the target."""

        distance = math.hypot(self.target_x - self.x, self.target_y - self.y)
        # A zero-end-speed cubic has a peak speed of 1.5 * distance / T.
        # Choose T so that the feed-forward peak stays at cruise_speed.
        self.segment_duration = max(
            1.0, 1.5 * distance / self.cruise_speed
        )
        self.x_coefficients = cubic_coefficients(
            self.x, self.target_x, self.segment_duration
        )
        self.y_coefficients = cubic_coefficients(
            self.y, self.target_y, self.segment_duration
        )
        self.segment_start_time = time.monotonic()
        self.state = "tracking"
        self.get_logger().info(
            "Waypoint %d/%d -> (%.3f, %.3f), T=%.2fs"
            % (
                self.waypoint_index + 1,
                len(self.waypoints),
                self.target_x,
                self.target_y,
                self.segment_duration,
            )
        )

    def control(self) -> None:
        """Timer callback implementing align-then-track closed-loop control."""

        if self.finished or not self.have_odometry:
            return

        if self.state == "aligning":
            heading_error = self.normalize_angle(self.segment_heading - self.yaw)
            if abs(heading_error) <= self.heading_tolerance:
                self.publish_velocity(0.0, 0.0)
                self.start_cubic_segment()
                return
            angular = self._clamp(
                self.heading_gain * heading_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.publish_velocity(0.0, angular)
            return

        if self.state != "tracking":
            return

        # 每一段都使用从 0 开始的相对时间，避免全局时间越来越大导致
        # t^2、t^3 项数值过大。这也是实验 7 文档建议的实现方式。
        elapsed = min(
            time.monotonic() - self.segment_start_time,
            self.segment_duration,
        )
        desired_x, desired_vx = evaluate_cubic(self.x_coefficients, elapsed)
        desired_y, desired_vy = evaluate_cubic(self.y_coefficients, elapsed)

        final_distance = math.hypot(self.target_x - self.x, self.target_y - self.y)
        if elapsed >= self.segment_duration and final_distance <= self.waypoint_tolerance:
            self.publish_velocity(0.0, 0.0)
            self.waypoint_index += 1
            self.begin_next_segment()
            return

        # “前馈 + 反馈”闭环控制：
        #   command_velocity = polynomial_velocity + Kp * position_error
        # 多项式速度负责沿计划轨迹前进，位置反馈负责修正打滑和仿真误差。
        command_x = desired_vx + self.position_gain * (desired_x - self.x)
        command_y = desired_vy + self.position_gain * (desired_y - self.y)
        planar_speed = math.hypot(command_x, command_y)

        if planar_speed < 1e-6:
            desired_heading = math.atan2(
                self.target_y - self.y, self.target_x - self.x
            )
        else:
            desired_heading = math.atan2(command_y, command_x)
        heading_error = self.normalize_angle(desired_heading - self.yaw)

        # command_x/command_y 是世界坐标中的二维速度，但差速机器人只能执行
        # 前向线速度 linear.x 和绕 z 轴角速度 angular.z。下面用 cos 将二维
        # 速度投影到机器人前向方向；若朝向误差太大，就先停止前进并转向。
        linear = min(planar_speed, self.max_linear_speed)
        linear *= max(0.0, math.cos(heading_error))
        if abs(heading_error) > 0.8:
            linear = 0.0
        angular = self._clamp(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.publish_velocity(linear, angular)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def publish_velocity(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.velocity_publisher.publish(message)

    def stop(self) -> None:
        """Publish several zero commands for a reliable Gazebo stop."""

        if not ROS_AVAILABLE:
            return
        for _ in range(3):
            self.publish_velocity(0.0, 0.0)

    def finish_mission(self) -> None:
        self.stop()
        self.state = "finished"
        self.finished = True
        self.timer.cancel()
        self.save_trajectory()
        self.get_logger().info("Goal reached; robot stopped in the red cell.")

    def save_trajectory(self) -> None:
        """Save measured odometry points for the required report plot."""

        if self.trajectory_saved or not self.trajectory:
            return
        output_path = os.path.abspath(os.path.expanduser(self.trajectory_file))
        try:
            with open(output_path, "w", newline="", encoding="utf-8") as output:
                writer = csv.writer(output)
                # Keep the file numeric-only so Lecture 3's visualization.py
                # can read it directly with numpy.loadtxt(..., delimiter=',').
                writer.writerows(self.trajectory)
            self.trajectory_saved = True
            self.get_logger().info("Trajectory saved to %s" % output_path)
        except OSError as error:
            self.get_logger().error("Could not save trajectory: %s" % error)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run until the goal is reached or the user presses Ctrl+C."""

    if not ROS_AVAILABLE:
        raise SystemExit(
            "ROS 2 is not installed. Source your ROS setup, or use --plan-only."
        )

    # 不使用永久的 rclpy.spin()，而是在任务完成条件外层调用 spin_once。
    # 这样机器人到达目标后程序能够自动保存轨迹、释放节点并正常退出。
    rclpy.init(args=args)
    robot: Optional[Turtlebot] = None
    try:
        robot = Turtlebot()
        while rclpy.ok() and not robot.finished:
            rclpy.spin_once(robot, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if robot is not None:
            robot.stop()
            robot.save_trajectory()
            robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if "--plan-only" in sys.argv:
        planned_path = build_mission_path(DEFAULT_START_CELL)
        print("start:", DEFAULT_START_CELL)
        print("path:", planned_path)
        print("moves:", len(planned_path))
        print(render_grid_path(DEFAULT_START_CELL, planned_path))
    else:
        main()
