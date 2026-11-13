#!/usr/bin/env python3
"""Kinematics library for the 4-DOF manipulator.

Provides forward/inverse kinematics, geometric Jacobian, and velocity
kinematics functions used by all ROS nodes.
"""

import logging

import numpy as np
from forward_kinematics import forward_kinematics_func, l1, l2, l3, l4

try:
    import rospy
except ImportError:
    rospy = None

# Map link lengths to DH-style constants used in the Jacobian derivation
d1 = l1  # base offset
a2, a3, a4 = l2, l3, l4  # link lengths of joints 2, 3, and wrist


def _log_warn(msg, *args):
    """Log via rospy when available, otherwise fall back to standard logging."""
    if rospy is not None:
        try:
            rospy.logwarn(msg, *args)
            return
        except Exception:
            pass
    logging.warning(msg, *args)


def normalize_angle(angle):
    """Wraps angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def get_full_kinematics(q):
    """
    Helper function based on YOUR forward_kinematics.py file.
    q = [q1, q2, q3, q4]  (here q4 == Joint_5 in Gazebo)
    """
    q1, q2, q3, q4 = q[0], q[1], q[2], q[3]

    # We only really need the EE position for IK, so call your Forward Kinematics:
    x, y, z = forward_kinematics_func(q1, q2, q3, q4)
    return np.array([x, y, z])


# -----------------------------
#   Geometric Jacobian (6x4)
#   (q4 here == Joint_5 in Gazebo)
# -----------------------------


def jacobian_geometric(q1, q2, q3, q4):
    """
    Returns the 6x4 geometric Jacobian for the 4-DOF robot.
    """
    s1, c1 = np.sin(q1), np.cos(q1)

    # Intermediate angle sums based on your FK logic
    delta = q2 - q3
    sigma = delta + q4

    sin_q2 = np.sin(q2)
    cos_q2 = np.cos(q2)
    sin_delta = np.sin(delta)
    cos_delta = np.cos(delta)
    sin_sigma = np.sin(sigma)
    cos_sigma = np.cos(sigma)

    common_sin = l4 * sin_sigma + l2 * sin_q2 + l3 * sin_delta
    common_cos = l4 * cos_sigma + l2 * cos_q2 + l3 * cos_delta
    wrist_cos = l4 * cos_sigma + l3 * cos_delta
    wrist_sin = l4 * sin_sigma + l3 * sin_delta

    J = np.array(
        [
            [s1 * common_sin, -c1 * common_cos, c1 * wrist_cos, -l4 * cos_sigma * c1],
            [-c1 * common_sin, -s1 * common_cos, s1 * wrist_cos, -l4 * cos_sigma * s1],
            [0.0, -common_sin, wrist_sin, -l4 * sin_sigma],
            [0.0, s1, -s1, s1],
            [0.0, -c1, c1, -c1],
            [1.0, 0.0, 0.0, 0.0],
        ]
    )

    return J


def jacobian_matrix(q):
    """
    Wrapper used by other code:
    q = [q1, q2, q3, q4]  ->  J (6x4)
    """
    q = np.asarray(q, dtype=float).ravel()
    assert q.size == 4, "q must be [q1, q2, q3, q4]"
    return jacobian_geometric(q[0], q[1], q[2], q[3])


def inverse_jacobian_matrix(q, damping=0.01):
    """
    Calculates the pseudo-inverse of the Jacobian.
    Uses Damped Least-Squares (J.T * (J * J.T + lambda^2 * I)^-1) when damping > 0,
    or the Moore-Penrose pseudoinverse when damping == 0.
    """
    J = jacobian_matrix(q)
    if damping == 0.0:
        return np.linalg.pinv(J)

    lambda_sq = damping**2
    identity = np.eye(6)
    return J.T @ np.linalg.inv(J @ J.T + lambda_sq * identity)


# -----------------------------------
#   Inverse Position Kinematics (IK)
# -----------------------------------


def inverse_kinematics_func(q0, X_des, max_iter=500, tolerance=1e-4, damping=0.01):
    """
    Solves Inverse Position Kinematics using Newton-Raphson with DLS.
    INCLUDES JOINT LIMITS: [-1.57, 1.57] rad
    """
    q_current = np.array(q0, dtype=float).ravel()

    # Normalize initial guess
    q_current = np.array([normalize_angle(q) for q in q_current])

    alpha = 0.5  # Learning rate

    # --- DEFINE JOINT LIMITS (Radians) ---
    # +/- 90 degrees = +/- 1.57 radians
    # You can adjust specific joints if needed (e.g., base might rotate more)
    q_min = np.array([-1.57, -1.57, -1.57, -1.57])
    q_max = np.array([1.57, 1.57, 1.57, 1.57])

    err_norm = 0.0
    for _i in range(max_iter):
        # 1. Forward kinematics
        x, y, z = forward_kinematics_func(q_current[0], q_current[1], q_current[2], q_current[3])
        X_current = np.array([x, y, z])

        # 2. Error
        e = X_des - X_current
        err_norm = np.linalg.norm(e)

        if err_norm < tolerance:
            return q_current

        # 3. 3x4 Linear Jacobian damped pseudoinverse (position-only IK; 4-DOF arm)
        J_lin = jacobian_matrix(q_current)[:3, :]
        if damping == 0.0:
            J_lin_inv = np.linalg.pinv(J_lin)
        else:
            J_lin_inv = J_lin.T @ np.linalg.inv(J_lin @ J_lin.T + (damping**2) * np.eye(3))

        # 4. Joint update
        q_delta = J_lin_inv.dot(e)

        # 6. Update joint angles
        q_current += alpha * q_delta

        # 7. Normalize angles
        q_current = np.array([normalize_angle(q) for q in q_current])

        # 8. --- APPLY JOINT LIMITS ---
        # Clamp values to be within [min, max]
        q_current = np.clip(q_current, q_min, q_max)

    # Only warn if error is significant
    if err_norm > 0.02:
        _log_warn(
            "IK did not converge. Final error = %.4f. (May be unreachable due to limits)",
            err_norm,
        )
    return q_current


# -----------------------------------
#   Inverse Velocity Kinematics
# -----------------------------------


def inverse_velocity_kinematics(q, V_des, damping=0.01):
    J_inv = inverse_jacobian_matrix(q, damping=damping)  # [4x6]
    q_dot = J_inv.dot(V_des)  # [4x1]
    return q_dot


def forward_velocity_kinematics(q, q_dot):
    q = np.asarray(q, dtype=float).ravel()
    q_dot = np.asarray(q_dot, dtype=float).ravel()
    J = jacobian_matrix(q)  # [6x4]
    V = J.dot(q_dot)  # [6x1]
    return V
