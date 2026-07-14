import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from forward_kinematics import forward_kinematics
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

        # Test Forward Kinematics
        self.test_case = [pi / 6, -pi / 3, -pi / 6]  # Joint angles in radians
        position = forward_kinematics(self.test_case)
        self.get_logger().info(f"Joint Angle Test Case: {self.test_case}")
        self.get_logger().info(f"End Position: {position}")

        # Create a timer to periodically call the publishing method
        self.create_timer(0.1, self.publish_joint_state)

    def publish_joint_state(self):
        # Set the timestamp
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        # Update the first three joint angles
        self.joint_msg.position[0:3] = array.array('d', self.test_case)
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
