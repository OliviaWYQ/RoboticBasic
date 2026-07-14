import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from inverse_kinematics import inverse_kinematics 
import array

class Manipulator(Node):
    def __init__(self):
        super().__init__('manipulator')
        self.get_logger().info("Press Ctrl + C to terminate")

        # Create a JointState publisher
        self.joint_pub = self.create_publisher(JointState, '/rx150/joint_states', 10)

        # Initialize the JointState message
        self.joint_msg = JointState()
        self.joint_msg.header = Header()
        self.joint_msg.name = ['waist', 'shoulder', 'elbow', 'wrist_angle',
                               'wrist_rotate', 'gripper', 'left_finger', 'right_finger']
        self.joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.026, -0.026]

        # Inverse Kinematics test
        self.test_case = [0.02165, 0.01250, 0.29721]  # Target position (x, y, z) in meters
        self.joint_angle = inverse_kinematics(self.test_case)
        self.get_logger().info(f"Test case (position): {self.test_case}")
        self.get_logger().info(f"Joint angles: {self.joint_angle}")

        # Create a timer that calls the publishing method every 0.1 seconds
        self.create_timer(0.1, self.publish_joint_state)

    def publish_joint_state(self):
        # Set the timestamp
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        # Convert joint_angle to array.array and update the position
        self.joint_msg.position[0:3] = array.array('d', self.joint_angle)
        # Publish the message
        self.joint_pub.publish(self.joint_msg)

def main(args=None):
    rclpy.init(args=args)
    manipulator = Manipulator()

    try:
        rclpy.spin(manipulator)
    except KeyboardInterrupt:
        print("Shutting down the manipulator node...")

if __name__ == '__main__':
    main()
