import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from Lecture5.Project5.forward_kinematics import forward_kinematics as fk_trig
from Lecture5.Project5.forward_kinematics_dh import forward_kinematics as fk_dh
from Lecture5.Project5.forward_kinematics_poe import forward_kinematics as fk_poe
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

        # Test Forward Kinematics - compare all three methods
        self.test_case = [pi / 6, -pi / 3, -pi / 6]  # Joint angles in radians

        pos_trig = fk_trig(self.test_case)
        pos_dh   = fk_dh(self.test_case)
        pos_poe  = fk_poe(self.test_case)

        self.get_logger().info(f"Joint Angle Test Case: {self.test_case}")
        self.get_logger().info(f"{'Method':<16} {'x':>10} {'y':>10} {'z':>10}")
        self.get_logger().info(f"{'-'*48}")
        self.get_logger().info(f"{'Trigonometry':<16} {pos_trig[0]:>10.4f} {pos_trig[1]:>10.4f} {pos_trig[2]:>10.4f}")
        self.get_logger().info(f"{'D-H Parameters':<16} {pos_dh[0]:>10.4f} {pos_dh[1]:>10.4f} {pos_dh[2]:>10.4f}")
        self.get_logger().info(f"{'PoE (Exponential)':<16} {pos_poe[0]:>10.4f} {pos_poe[1]:>10.4f} {pos_poe[2]:>10.4f}")

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
