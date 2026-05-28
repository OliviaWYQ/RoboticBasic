#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot(Node):
    def __init__(self):
        super().__init__("turtlebot_move")
        self.get_logger().info("Press Ctrl + C to terminate")
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Tunable parameters
        linear_speed  = 0.5        # m/s
        angular_speed = pi / 2     # rad/s  (90 deg/s)
        move_distance = 4.0        # meters
        turn_angle    = pi / 2     # radians (90 deg)

        # Ticks at 10 Hz needed for each phase
        move_ticks = int(move_distance / linear_speed  / 0.1)  # 80 ticks = 8 s
        turn_ticks = int(turn_angle    / angular_speed / 0.1)  # 10 ticks = 1 s

        # Build sequence: 4 × (move forward, turn left)
        self.sequence = []
        for _ in range(4):
            self.sequence.append((linear_speed, 0.0,          move_ticks))
            self.sequence.append((0.0,          angular_speed, turn_ticks))

        self.step  = 0
        self.count = 0
        self.timer = self.create_timer(0.1, self.run)  # 10 Hz

    def run(self):
        vel = Twist()

        if self.step >= len(self.sequence):
            self.vel_pub.publish(vel)  # publish zero velocity to stop
            self.get_logger().info("Square trajectory complete!")
            self.timer.cancel()
            raise SystemExit

        linear_x, angular_z, duration = self.sequence[self.step]
        vel.linear.x  = linear_x
        vel.angular.z = angular_z
        self.vel_pub.publish(vel)
        self.count += 1

        if self.count >= duration:
            self.step  += 1
            self.count  = 0

def main(args=None):
    rclpy.init(args=args)
    turtlebot = Turtlebot()

    try:
        rclpy.spin(turtlebot)
    except KeyboardInterrupt:
        print("Ctrl + C detected. Exiting...")
    except SystemExit:
        pass
    finally:
        turtlebot.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
