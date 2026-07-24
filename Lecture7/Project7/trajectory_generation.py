#!/usr/bin/env python3

"""使用三次多项式生成轨迹，并控制 Turtlebot 依次经过所有路标点。

整体思路：
1. 把相邻路标点之间看作一段独立轨迹，每段时间都从 0 重新开始。
2. 分别为 x(t)、y(t) 求三次多项式，使每段起点和终点的位置、速度满足约束。
3. 以 10 Hz 计算当前期望位置和速度，再加入位置误差反馈。
4. 将平面速度方向转换为机器人的线速度和角速度，并发布到 /cmd_vel。
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class Turtlebot(Node):
    """负责轨迹生成、轨迹跟踪以及里程计记录的 ROS 2 节点。"""

    def __init__(self):
        super().__init__('turtlebot_controller')
        self.get_logger().info(
            "Trajectory generation started. Press Ctrl+C to terminate."
        )

        # 向 /cmd_vel 发布控制命令，从 /odom 获取机器人的实际位姿。
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # pose 保存里程计反馈的实际位姿；vel 是重复使用的速度消息。
        # trajectory 用来记录实际运动轨迹，程序退出时保存成 CSV 文件。
        self.pose = Pose2D()
        self.vel = Twist()
        self.trajectory = []

        # 机器人将按照列表顺序访问这些路标点。
        # 最后重复一次原点，使最后一个轨迹段能够把速度平滑降到 0。
        self.waypoints = [
            [0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
            [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
            [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]
        ]

        # 每段都使用相对时间 [0, segment_duration]，符合实验对数值稳定性的建议。
        # 3 秒走完一段 0.5 m 左右的轨迹，速度不会太快，也给转向留出时间。
        self.timer_period = 0.1       # 控制周期 0.1 秒，即 10 Hz
        self.segment_duration = 3.0  # 每段轨迹的持续时间 T
        self.segment_index = 0       # 下一次要处理的路标点下标
        self.segment_time = 0.0      # 当前轨迹段内的相对时间 t

        # 第一段从机器人初始位置 (0, 0) 开始，并假设初始速度为 0。
        # 后续每准备一段，这两个变量会更新为上一段的终点和终点速度。
        self.previous_waypoint = np.array([0.0, 0.0], dtype=float)
        self.previous_velocity = np.zeros(2, dtype=float)

        # coefficients 是 2x4 矩阵：第一行是 x(t) 的系数，第二行是 y(t)。
        self.coefficients = None
        self.current_target = None  # 当前轨迹段的终点
        self.finished = False       # 是否已经处理完全部路标点

        # 控制器参数。position_gain 修正位置偏差，heading_gain 修正朝向偏差。
        # 限幅可防止误差较大时产生不安全的速度命令。
        self.position_gain = 1.0       # 位置 P 控制增益
        self.heading_gain = 3.0        # 航向 P 控制增益
        self.max_linear_speed = 0.40   # 最大线速度，单位 m/s
        self.max_angular_speed = 1.50  # 最大角速度，单位 rad/s

        # 先计算第一段多项式，再启动 10 Hz 定时控制器。
        self.prepare_next_segment()
        self.timer = self.create_timer(
            self.timer_period, self.timed_controller_callback
        )

    def prepare_next_segment(self):
        """计算下一段 x(t)、y(t) 的三次多项式系数。"""

        # 没有剩余路标点时，发布零速度、停止定时器、保存轨迹并退出。
        if self.segment_index >= len(self.waypoints):
            self.coefficients = None
            self.finished = True
            self.stop()
            if hasattr(self, 'timer'):
                self.timer.cancel()
            self.save_trajectory()
            self.get_logger().info("All waypoints completed. Exiting...")
            self.destroy_node()
            rclpy.shutdown()

        # 当前段从 previous_waypoint 运动到 target。
        target = np.asarray(self.waypoints[self.segment_index], dtype=float)

        # 用中心差分估计内部路标点处的速度：
        #
        #   v_i = (p_(i+1) - p_(i-1)) / (2T)
        #
        # 本段的终点速度会直接作为下一段的起点速度，因此相邻两段在路标点
        # 处速度相同，轨迹满足一阶连续（C1 连续），机器人不会突然改变速度。
        if self.segment_index + 1 < len(self.waypoints):
            following = np.asarray(
                self.waypoints[self.segment_index + 1], dtype=float
            )
            target_velocity = (
                following - self.previous_waypoint
            ) / (2.0 * self.segment_duration)
        else:
            # 最后一个点没有后继点，规定终点速度为 0，使机器人停下。
            target_velocity = np.zeros(2, dtype=float)

        # x 和 y 方向相互独立，各求一组三次多项式系数。
        # 每组系数都满足：起点位置、起点速度、终点位置和终点速度。
        coefficients_x = self.polynomial_time_scaling_3rd_order(
            self.previous_waypoint[0],
            self.previous_velocity[0],
            target[0],
            target_velocity[0],
            self.segment_duration,
        )
        coefficients_y = self.polynomial_time_scaling_3rd_order(
            self.previous_waypoint[1],
            self.previous_velocity[1],
            target[1],
            target_velocity[1],
            self.segment_duration,
        )

        # 每一行按 [a0, a1, a2, a3] 保存，方便后面用矩阵乘法求值。
        self.coefficients = np.vstack((coefficients_x, coefficients_y))
        self.current_target = target

        # 新轨迹段使用相对时间，所以每段开始时都把 t 归零。
        self.segment_time = 0.0

        # 保存本段终点状态，下一次调用时它就是下一段的起点状态。
        self.previous_waypoint = target
        self.previous_velocity = target_velocity
        self.segment_index += 1

    def timed_controller_callback(self):
        """以 10 Hz 计算当前轨迹状态，并发布线速度和角速度。"""

        # 尚未得到系数或整个任务已经结束时，不再计算控制命令。
        if self.coefficients is None or self.finished:
            return

        # 把时间限制在 [0, T] 内，避免定时器的微小误差使多项式越过终点。
        t = min(self.segment_time, self.segment_duration)

        # 若 p(t) = a0 + a1*t + a2*t^2 + a3*t^3，则：
        # p(t) 由 time_terms 求出，p'(t) 由 derivative_terms 求出。
        time_terms = np.array([1.0, t, t * t, t * t * t])
        derivative_terms = np.array([0.0, 1.0, 2.0 * t, 3.0 * t * t])

        # 2x4 系数矩阵分别乘以时间向量，得到 [x, y] 和 [vx, vy]。
        desired_position = self.coefficients @ time_terms
        desired_velocity = self.coefficients @ derivative_terms

        # 期望位置减去里程计位置，得到机器人当前的位置误差。
        position_error = desired_position - np.array(
            [self.pose.x, self.pose.y], dtype=float
        )

        # “前馈 + 反馈”控制：
        # - desired_velocity 是多项式给出的前馈速度，负责沿轨迹前进；
        # - Kp * position_error 把机器人拉回期望位置，防止误差不断累积。
        tracking_velocity = (
            desired_velocity + self.position_gain * position_error
        )
        tracking_speed = float(np.linalg.norm(tracking_velocity))

        # atan2(vy, vx) 得到期望速度方向，也就是机器人应该朝向的角度。
        if tracking_speed > 1e-9:
            desired_heading = math.atan2(
                tracking_velocity[1], tracking_velocity[0]
            )
            heading_error = self._normalize_angle(
                desired_heading - self.pose.theta
            )
        else:
            heading_error = 0.0

        # Turtlebot 不能直接执行世界坐标系中的 [vx, vy]，只能接收：
        # - linear.x：沿机器人正前方的线速度；
        # - angular.z：绕竖直轴的角速度。
        #
        # cos(heading_error) 将期望平面速度投影到机器人正前方。当朝向误差
        # 较大时先减速转向，可减少拐角处“切弯”；max(0, ...) 禁止机器人倒车。
        linear_speed = tracking_speed * max(0.0, math.cos(heading_error))

        # np.clip 对速度限幅；角速度采用简单的 P 控制 omega = K * 角度误差。
        self.vel.linear.x = float(np.clip(
            linear_speed, 0.0, self.max_linear_speed
        ))
        self.vel.angular.z = float(np.clip(
            self.heading_gain * heading_error,
            -self.max_angular_speed,
            self.max_angular_speed,
        ))
        self.vel_pub.publish(self.vel)

        # 时间前进一个控制周期。当前段完成后，准备下一段并把相对时间归零。
        self.segment_time += self.timer_period
        if self.segment_time > self.segment_duration + 1e-9:
            self.prepare_next_segment()

    def polynomial_time_scaling_3rd_order(
        self, p_start, v_start, p_end, v_end, T
    ):
        """返回满足起止位置和速度约束的三次多项式系数。"""

        # 输入：p_start、v_start 为起点位置和速度；
        #       p_end、v_end 为终点位置和速度；T 为该段持续时间。
        # 输出：[a0, a1, a2, a3]，对应
        #       p(t) = a0 + a1*t + a2*t^2 + a3*t^3。
        if T <= 0.0:
            raise ValueError("T must be greater than zero")

        # 将四个边界条件代入多项式：
        #   p(0)  = p_start,  p'(0) = v_start
        #   p(T)  = p_end,    p'(T) = v_end
        # 解这四个方程即可得到下面四个系数。
        a0 = float(p_start)
        a1 = float(v_start)
        a2 = (
            3.0 * (p_end - p_start) / (T * T)
            - (2.0 * v_start + v_end) / T
        )
        a3 = (
            2.0 * (p_start - p_end) / (T * T * T)
            + (v_start + v_end) / (T * T)
        )
        return np.array([a0, a1, a2, a3], dtype=float)

    @staticmethod
    def _normalize_angle(angle):
        """把角度归一化到 [-pi, pi]，保证机器人选择较短的转向方向。"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop(self):
        """发布全零 Twist 消息，使机器人停止。"""
        stop_msg = Twist()
        self.vel_pub.publish(stop_msg)
        self.get_logger().info("Robot stopped.")

    def save_trajectory(self):
        """将里程计记录的实际 x、y 坐标保存到 trajectory.csv。"""
        np.savetxt(
            'trajectory.csv',
            np.array(self.trajectory),
            fmt='%f',
            delimiter=',',
        )
        print("Trajectory saved to trajectory.csv")

    def odom_callback(self, msg):
        """读取里程计，更新实际位姿并记录轨迹点。"""

        # ROS 的姿态是四元数，这里只需要平面运动的偏航角 yaw。
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.trajectory.append([self.pose.x, self.pose.y])


def main(args=None):
    """初始化 ROS 2 节点，并持续处理定时器与里程计回调。"""
    rclpy.init(args=args)
    turtlebot = Turtlebot()

    try:
        rclpy.spin(turtlebot)
    except KeyboardInterrupt:
        print("Ctrl + C detected. Exiting...")
    finally:
        # 无论正常结束还是 Ctrl+C，先保存轨迹，再尝试停车。
        turtlebot.save_trajectory()
        try:
            turtlebot.stop()
        except Exception:
            pass
        try:
            turtlebot.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
