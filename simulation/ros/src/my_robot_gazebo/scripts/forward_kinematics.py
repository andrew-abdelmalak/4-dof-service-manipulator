#!/usr/bin/env python3

import numpy as np

# Define the robot's physical parameters (link lengths in meters)
l1 = 0.04355
l2 = 0.140
l3 = 0.133
l4 = 0.109


def transformation_func(theta, d, a, alpha):
    """
    Calculates the Total Homogeneous Transformation Matrix (T)
    based on DH parameters.
    """
    T = np.array(
        [
            [
                np.cos(theta),
                -np.sin(theta) * np.cos(alpha),
                np.sin(theta) * np.sin(alpha),
                a * np.cos(theta),
            ],
            [
                np.sin(theta),
                np.cos(theta) * np.cos(alpha),
                -np.cos(theta) * np.sin(alpha),
                a * np.sin(theta),
            ],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )
    return T


def forward_kinematics_func(q1, q2, q3, q4):
    """
    Calculates the end-effector's position (x, y, z) for the 4-DOF arm.
    """
    T01 = transformation_func(q1, l1, 0, np.pi / 2)
    T12 = transformation_func(np.pi / 2 + q2, 0, l2, np.pi)
    T23 = transformation_func(q3, 0, l3, np.pi)
    T34 = transformation_func(q4, 0, l4, 0)

    T04 = np.dot(T01, np.dot(T12, np.dot(T23, T34)))

    x = T04[0, 3]
    y = T04[1, 3]
    z = T04[2, 3]

    return [x, y, z]


if __name__ == "__main__":
    ee_position = forward_kinematics_func(0.5, 0.5, 0.5, 0.5)
    print(f"Calculated 4-DOF Position (x, y, z): {ee_position}")
