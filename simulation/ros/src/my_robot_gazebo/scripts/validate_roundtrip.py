#!/usr/bin/env python3
"""
validate_roundtrip.py

Offline test (no Gazebo, no /end_effector_pose) that validates:
- MS2 forward kinematics
- MS3 forward velocity kinematics
- MS3 inverse position kinematics
- MS3 inverse velocity kinematics

Round-trip:
  (q, q_dot) -> (x, V) -> (q_rec, q_dot_rec)
and compare input vs output.
"""

import numpy as np

from forward_kinematics import forward_kinematics_func
from kinematics import (
    inverse_kinematics_func,
    forward_velocity_kinematics,
    inverse_velocity_kinematics,
)


def print_vec(name, v):
    v = np.asarray(v).ravel()
    print(f"{name}: " + "[" + ", ".join(f"{x: .6f}" for x in v) + "]")


def single_test(q, q_dot):
    q = np.asarray(q, dtype=float).ravel()
    q_dot = np.asarray(q_dot, dtype=float).ravel()

    print("=== New test case ===")
    print_vec("q      (input)", q)
    print_vec("q_dot  (input)", q_dot)

    # 1) Forward position (MS2)
    x = np.array(forward_kinematics_func(q[0], q[1], q[2], q[3]))
    print_vec("x = FK(q)", x)

    # 2) Forward velocity (MS3)
    V = forward_velocity_kinematics(q, q_dot)
    print_vec("V = J(q) * q_dot", V)

    # 3) Inverse position: ask IK to recover q from x
    #    Use the true q as initial guess so it converges back to it.
    q_rec = inverse_kinematics_func(q, x)
    print_vec("q_rec from IK(x)", q_rec)

    # 4) Inverse velocity: recover q_dot from (q_rec, V)
    q_dot_rec = inverse_velocity_kinematics(q_rec, V)
    print_vec("q_dot_rec from J^+ * V", q_dot_rec)

    # 5) Errors
    dq = q_rec - q
    dq_dot = q_dot_rec - q_dot

    print_vec("q_rec - q", dq)
    print_vec("q_dot_rec - q_dot", dq_dot)
    print(f"||q_rec - q||       = {np.linalg.norm(dq):.6e}")
    print(f"||qdot_rec - qdot|| = {np.linalg.norm(dq_dot):.6e}")
    print("")


def main():
    # A few sample joint configs (in radians) and velocities (rad/s)
    test_cases = [
        # q,                       q_dot
        ([0.0, 0.0, 0.0, 0.0],     [0.1, 0.0, 0.0, 0.0]),
        ([0.2, 0.4, -0.3, 0.1],    [0.0, 0.2, -0.1, 0.05]),
        ([-0.3, 0.6, 0.2, -0.2],   [0.1, -0.15, 0.05, 0.0]),
    ]

    for q, q_dot in test_cases:
        single_test(q, q_dot)


if __name__ == "__main__":
    main()


