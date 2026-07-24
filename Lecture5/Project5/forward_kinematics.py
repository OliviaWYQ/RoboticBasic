import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]

    link1z = 0.065   # base to shoulder (Z)
    link2z = 0.039   # shoulder offset (Z)
    link3x = 0.050   # shoulder to elbow (X)
    link3z = 0.150   # shoulder to elbow (Z)
    link4x = 0.150   # elbow to end effector (X)

    joint1 = joints[0]  # waist: rotation about Z-axis
    joint2 = joints[1]  # shoulder: rotation about Y-axis (pitch)
    joint3 = joints[2]  # elbow: rotation about Y-axis (pitch)

    # Step 1: compute position in the XZ plane (arm's local plane)
    # Shoulder->Elbow vector rotated by joint2:
    #   (link3x, link3z) * rotation_matrix_y(joint2)
    #   x' = link3x*cos(j2) + link3z*sin(j2)
    #   z' = -link3x*sin(j2) + link3z*cos(j2)
    #
    # Elbow->End vector rotated by joint2 + joint3:
    #   (link4x, 0) * rotation_matrix_y(joint2+joint3)
    #   x'' = link4x*cos(j2+j3)
    #   z'' = -link4x*sin(j2+j3)

    x_local = (link3x * cos(joint2) + link3z * sin(joint2)
               + link4x * cos(joint2 + joint3))

    z_local = (link1z + link2z
               - link3x * sin(joint2) + link3z * cos(joint2)
               - link4x * sin(joint2 + joint3))

    # Step 2: rotate the local XZ plane about Z-axis by joint1
    x = x_local * cos(joint1)
    y = x_local * sin(joint1)
    z = z_local

    return [x, y, z]
