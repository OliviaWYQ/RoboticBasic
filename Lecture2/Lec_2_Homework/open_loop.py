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
        self.count = 0
        self.max_count = 50  # publish 50 times
        self.timer = self.create_timer(0.1, self.run)  # 10 Hz

    def run(self):
        vel = Twist()
        if self.count < self.max_count:
            vel.linear.x = 0.5
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)
            self.count += 1
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)
            self.get_logger().info("Done publishing velocity")
            self.timer.cancel()  # stop timer after done

def main(args=None):
    rclpy.init(args=args)
    turtlebot = Turtlebot()

    try:
        rclpy.spin(turtlebot)
    except KeyboardInterrupt:
        print("Ctrl + C detected. Exiting...")


if __name__ == "__main__":
    main()
