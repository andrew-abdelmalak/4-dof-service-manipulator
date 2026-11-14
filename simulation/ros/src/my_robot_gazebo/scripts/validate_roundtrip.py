#!/usr/bin/env python3
"""Offline roundtrip validation for kinematics and Jacobian consistency.

Validates:
- Forward position kinematics (including zero-config height 0.42555 m)
- Analytical linear Jacobian vs central finite differences
- Warm-start and cold-start (q0 = 0) inverse position kinematics convergence
- Damped (lambda = 0.01) and undamped (pinv, lambda = 0) velocity roundtrip
"""

import numpy as np
from forward_kinematics import forward_kinematics_func
from kinematics import (
    forward_velocity_kinematics,
    inverse_kinematics_func,
    inverse_velocity_kinematics,
    jacobian_matrix,
)


def print_vec(name, v):
    v = np.asarray(v).ravel()
    print(f"{name}: [" + ", ".join(f"{x: .6f}" for x in v) + "]")


def finite_diff_linear_jacobian(q, eps=1e-7):
    J_fd = np.zeros((3, 4))
    for i in range(4):
        dq = np.zeros(4)
        dq[i] = eps
        fp = np.asarray(forward_kinematics_func(*(q + dq)), dtype=float)
        fm = np.asarray(forward_kinematics_func(*(q - dq)), dtype=float)
        J_fd[:, i] = (fp - fm) / (2.0 * eps)
    return J_fd


def single_test(q, q_dot):
    q = np.asarray(q, dtype=float).ravel()
    q_dot = np.asarray(q_dot, dtype=float).ravel()

    print("=== New test case ===")
    print_vec("q      (input)", q)
    print_vec("q_dot  (input)", q_dot)

    # 1) Forward position
    x = np.asarray(forward_kinematics_func(q[0], q[1], q[2], q[3]), dtype=float)
    print_vec("x = FK(q)", x)

    # 2) Analytical vs numerical finite-difference Jacobian
    J_analytical = jacobian_matrix(q)[:3, :]
    J_fd = finite_diff_linear_jacobian(q)
    jac_err = float(np.max(np.abs(J_analytical - J_fd)))
    print(f"max |J_lin - J_fd|  = {jac_err:.6e}")
    assert jac_err < 1e-8, f"Jacobian mismatch: {jac_err}"

    # 3) Forward velocity
    V = forward_velocity_kinematics(q, q_dot)
    print_vec("V = J(q) * q_dot", V)

    # 4) Warm-start IK consistency
    q_warm = inverse_kinematics_func(q, x)
    dq_warm = q_warm - q
    assert np.linalg.norm(dq_warm) < 1e-12

    # 5) Cold-start IK convergence from q0 = [0, 0, 0, 0]
    q_cold = inverse_kinematics_func(np.zeros(4), x)
    x_cold = np.asarray(forward_kinematics_func(*q_cold), dtype=float)
    pos_err_cold = float(np.linalg.norm(x - x_cold))
    print_vec("q_cold from IK(0,x)", q_cold)
    print(f"||x - FK(q_cold)||  = {pos_err_cold:.6e} m")
    assert pos_err_cold < 1e-4, f"Cold-start IK failed: {pos_err_cold}"

    # 6) Inverse velocity: damped (lambda=0.01) and undamped (pinv, lambda=0)
    q_dot_dls = inverse_velocity_kinematics(q_warm, V, damping=0.01)
    q_dot_pinv = inverse_velocity_kinematics(q_warm, V, damping=0.0)
    dq_dot_dls = q_dot_dls - q_dot
    dq_dot_pinv = q_dot_pinv - q_dot

    print(f"||q_warm - q||            = {np.linalg.norm(dq_warm):.6e} rad")
    print(f"||qdot_dls - qdot||       = {np.linalg.norm(dq_dot_dls):.6e} rad/s")
    print(f"||qdot_pinv - qdot||      = {np.linalg.norm(dq_dot_pinv):.6e} rad/s")

    assert np.linalg.norm(dq_dot_dls) < 1e-2
    assert np.linalg.norm(dq_dot_pinv) < 1e-14
    print("")


def main():
    x_zero = np.asarray(forward_kinematics_func(0.0, 0.0, 0.0, 0.0), dtype=float)
    assert np.allclose(x_zero, [0.0, 0.0, 0.42555], atol=1e-12)

    test_cases = [
        ([0.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.0]),
        ([0.2, 0.4, -0.3, 0.1], [0.0, 0.2, -0.1, 0.05]),
        ([-0.3, 0.6, 0.2, -0.2], [0.1, -0.15, 0.05, 0.0]),
    ]

    for q, q_dot in test_cases:
        single_test(q, q_dot)

    print("ALL ROUNDTRIP CHECKS PASSED")


if __name__ == "__main__":
    main()
