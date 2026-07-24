import rclpy
from rclpy.node import Node
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from Lecture6.Project6.inverse_kinematics_analytic import inverse_kinematics as ik_analytic
from Lecture6.Project6.inverse_kinematics_numerical import inverse_kinematics as ik_numerical
import array

USE_ANALYTIC = False  # True=analytic, False=numerical

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
        self.test_case =  [0.15, 0.0, 0.25] # Target position (x, y, z) in meters

        joint_analytic  = ik_analytic(self.test_case)
        joint_numerical = ik_numerical(self.test_case)

        self.get_logger().info(f"Test case (target position): {self.test_case}")
        self.get_logger().info(f"{'Method':<22} {'waist':>8} {'shoulder':>10} {'elbow':>8}")
        self.get_logger().info(f"{'-'*50}")
        self.get_logger().info(f"{'Analytic':<22} {joint_analytic[0]:>8.4f} {joint_analytic[1]:>10.4f} {joint_analytic[2]:>8.4f}")
        self.get_logger().info(f"{'Numerical':<22} {joint_numerical[0]:>8.4f} {joint_numerical[1]:>10.4f} {joint_numerical[2]:>8.4f}")

        # Use analytic solution for publishing to robot_state_publisher
        self.joint_angle = joint_analytic if USE_ANALYTIC else joint_numerical
        self.get_logger().info(f"Publishing: {'Analytic' if USE_ANALYTIC else 'Numerical'}")

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
