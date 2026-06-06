#!/usr/bin/env python3

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class Controller:
    """PD controller with wraparound error for angular control."""

    def __init__(self, P: float=0.0, D: float=0.0, set_point: float=0.0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point
        self.previous_error = 0.0

    def update(self, current_value: float) -> float:
        """
        TODO:
        1. Compute angular error (set_point - current_value), wrapped to [-π, π].
        2. Apply PD formula to compute control output.
        3. Update previous_error.
        4. Return control output (omega).
        """
        pass

    def setPoint(self, set_point: float):
        """Set reference heading and reset error history."""
        self.set_point = set_point
        self.previous_error = 0.0

    def setPD(self, P: float=0.0, D: float=0.0):
        """Update PD gains."""
        self.Kp = P
        self.Kd = D


class Turtlebot(Node):
    """ROS2 node for sequential waypoint navigation with trajectory logging."""

    def __init__(self):
        super().__init__('turtlebot_closed_loop')
        self.get_logger().info("Press Ctrl + C to terminate")
        self.pose = Pose2D()
        self.trajectory = []

        self.vel = Twist()
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.rate_hz = 10
        self.rate_dt = 1.0 / self.rate_hz
        self.pid_theta = Controller(P=1.0, D=0.1)
        self.get_logger().info('Initialized closed-loop navigator')

    def run(self):
        """
        TODO:
        1. Define a list of waypoints (e.g., square path).
        2. For each waypoint, call move_to_point(tx, ty).
        3. After all waypoints, save trajectory and stop the robot.
        """
        pass

    def move_to_point(self, tx: float, ty: float):
        """
        TODO:
        1. Compute distance and heading to the target point.
        2. If heading error > angle_threshold, rotate in place using PD controller.
        3. If heading is aligned, move forward while maintaining heading.
        4. Stop when within distance_threshold of the target point.
        """
        pass

    def stop(self):
        """Stop the robot by publishing zero velocities."""
        stop_msg = Twist()
        self.vel_pub.publish(stop_msg)
        time.sleep(0.1)

    def save_to_csv(self):
        """Save trajectory to a csv file. """
        np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
        print("Trajectory saved to trajectory.csv")

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry and log position."""
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.trajectory.append([self.pose.x, self.pose.y])


def main(args=None):
    rclpy.init(args=args)
    turtlebot = Turtlebot()

    try:
        turtlebot.run()
    except KeyboardInterrupt:
        print("Ctrl + C detected. Exiting...")
    finally:
        turtlebot.save_trajectory()


if __name__ == '__main__':
    main()
