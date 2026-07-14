#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class Turtlebot(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.get_logger().info("Trajectory generation started. Press Ctrl+C to terminate.")

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timed_controller_callback)

        self.pose = Pose2D()
        self.vel = Twist()
        self.trajectory = []

        self.waypoints = [
            [0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
            [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
            [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]
        ]

    def prepare_next_segment(self):
        """ Prepare coefficients for the next trajectory segment.
        TODO:
        1. If all segments completed, stop the timed controller.
        2. Get current and next waypoints.
        3. Calculate position and velocity coefficients using polynomial time scaling.
        4. Update previous waypoint and velocity for next segment.
        """
        pass

    def timed_controller_callback(self):
        """ send velocity commands at 10 Hz based on trajectory segments.
        TODO:
        1. If coefficients are not initialized, skip this step.
        2. If current segment is completed, prepare next segment.
        3. Calculate position and velocity at current step using polynomial coefficients.
        4. Compute heading error and apply P controller to get angular velocity.
        5. Publish velocity command.
        """
        pass

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        """ Calculate coefficients for a 3rd order polynomial given boundary conditions """
        # Input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in seconds)
        # Output: the coefficients of this polynomial
        pass

    def stop(self):
        """Stop the robot by publishing zero velocities."""
        stop_msg = Twist()
        self.vel_pub.publish(stop_msg)
        self.get_logger().info("Robot stopped.")

    def save_trajectory(self):
        """Save trajectory to a csv file. """
        np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')
        print("Trajectory saved to trajectory.csv")

    def odom_callback(self, msg):
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
        rclpy.spin(turtlebot)
    except KeyboardInterrupt:
        print("Ctrl + C detected. Exiting...")
    finally:
        turtlebot.save_trajectory()


if __name__ == '__main__':
    main()
