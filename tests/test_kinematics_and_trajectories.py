"""Unit and integration tests for 4-DOF kinematics, planners, and CSV trajectories."""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = REPO_ROOT / "simulation" / "ros" / "src" / "my_robot_gazebo" / "scripts"
ARDUINO_DIR = REPO_ROOT / "hardware" / "arduino"
TRAJECTORIES_DIR = ARDUINO_DIR / "trajectories"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import forward_kinematics as fk  # noqa: E402
import kinematics as kin  # noqa: E402
import trajectory_planner as tp  # noqa: E402


def _load_send_trajectory():
    spec = importlib.util.spec_from_file_location(
        "send_trajectory", ARDUINO_DIR / "send_trajectory.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_zero_config_forward_kinematics() -> None:
    """Zero joint angles place the end-effector straight up at l1+l2+l3+l4."""
    pos = fk.forward_kinematics_func(0.0, 0.0, 0.0, 0.0)
    expected = np.array([0.0, 0.0, fk.l1 + fk.l2 + fk.l3 + fk.l4])
    assert np.allclose(pos, expected, atol=1e-12)
    assert math.isclose(expected[2], 0.42555, rel_tol=0.0, abs_tol=1e-12)


@pytest.mark.parametrize(
    ("q", "expected"),
    [
        ([0.2, 0.4, -0.3, 0.1], [-0.21403838, -0.04338809, 0.35016419]),
        ([-0.3, 0.6, 0.2, -0.2], [-0.14568571, 0.04506640, 0.38842498]),
    ],
)
def test_benchmark_forward_kinematics(q: list[float], expected: list[float]) -> None:
    """Verify FK coordinates reported in the paper benchmark table."""
    pos = np.asarray(fk.forward_kinematics_func(*q))
    assert np.allclose(pos, expected, atol=1e-7)


def test_planar_rotation_forward_kinematics() -> None:
    """Rotating q2 by pi/2 extends arm horizontally along -x (q1=0) or -y (q1=pi/2)."""
    reach = fk.l2 + fk.l3 + fk.l4
    pos_x = fk.forward_kinematics_func(0.0, math.pi / 2, 0.0, 0.0)
    assert np.allclose(pos_x, [-reach, 0.0, fk.l1], atol=1e-12)

    pos_y = fk.forward_kinematics_func(math.pi / 2, math.pi / 2, 0.0, 0.0)
    assert np.allclose(pos_y, [0.0, -reach, fk.l1], atol=1e-12)


def test_dh_transformation_is_se3() -> None:
    """Each DH transform must have an orthonormal rotation block with det = +1."""
    T = fk.transformation_func(0.35, 0.04355, 0.14, math.pi / 2)
    R = T[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-12)
    assert np.allclose(T[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12)


def test_analytical_linear_jacobian_matches_finite_differences() -> None:
    """Analytical 3x4 linear Jacobian agrees with central finite differences."""
    rng = np.random.default_rng(42)
    eps = 1e-7
    for q in rng.uniform(-1.3, 1.3, size=(20, 4)):
        J_lin = kin.jacobian_matrix(q)[:3, :]
        J_fd = np.zeros((3, 4))
        for j in range(4):
            dq = np.zeros(4)
            dq[j] = eps
            f_plus = np.asarray(fk.forward_kinematics_func(*(q + dq)))
            f_minus = np.asarray(fk.forward_kinematics_func(*(q - dq)))
            J_fd[:, j] = (f_plus - f_minus) / (2.0 * eps)
        assert np.max(np.abs(J_lin - J_fd)) < 1e-8


def test_jacobian_angular_rows_at_zero_and_offset() -> None:
    """Base joint rotates around +z; pitch joints rotate around base-rotated -y."""
    J_zero = kin.jacobian_matrix([0.0, 0.0, 0.0, 0.0])
    assert np.allclose(J_zero[3:, 0], [0.0, 0.0, 1.0], atol=1e-12)
    assert np.allclose(J_zero[3:, 1], [0.0, -1.0, 0.0], atol=1e-12)
    assert np.allclose(J_zero[3:, 2], [0.0, 1.0, 0.0], atol=1e-12)
    assert np.allclose(J_zero[3:, 3], [0.0, -1.0, 0.0], atol=1e-12)


@pytest.mark.parametrize(
    "q_target",
    [
        [0.0, 0.0, 0.0, 0.0],
        [0.2, 0.4, -0.3, 0.1],
        [-0.3, 0.6, 0.2, -0.2],
        [0.7, -0.45, 0.35, -0.25],
    ],
)
def test_ik_cold_and_warm_start_convergence(q_target: list[float]) -> None:
    """Position IK converges below 1e-4 m from cold start and exact q from warm start."""
    X_des = np.asarray(fk.forward_kinematics_func(*q_target))
    q_cold = kin.inverse_kinematics_func(np.zeros(4), X_des)
    X_cold = np.asarray(fk.forward_kinematics_func(*q_cold))
    assert np.linalg.norm(X_des - X_cold) < 1e-4
    assert np.all(q_cold >= -1.57) and np.all(q_cold <= 1.57)

    q_warm = kin.inverse_kinematics_func(q_target, X_des)
    assert np.linalg.norm(q_warm - np.asarray(q_target)) < 1e-12


def test_ik_respects_joint_limits_on_unreachable_target() -> None:
    """Unreachable targets still return joint angles clipped within [-1.57, 1.57] rad."""
    q_sol = kin.inverse_kinematics_func(np.zeros(4), np.array([2.0, 2.0, 2.0]))
    assert np.all(q_sol >= -1.57) and np.all(q_sol <= 1.57)


@pytest.mark.parametrize(
    ("q", "q_dot", "expected_dls_bound"),
    [
        ([0.0, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.0], 2e-5),
        ([0.2, 0.4, -0.3, 0.1], [0.0, 0.2, -0.1, 0.05], 1e-2),
        ([-0.3, 0.6, 0.2, -0.2], [0.1, -0.15, 0.05, 0.0], 1e-2),
    ],
)
def test_velocity_kinematics_damped_and_pinv_roundtrip(
    q: list[float], q_dot: list[float], expected_dls_bound: float
) -> None:
    """Both DLS (lambda=0.01) and undamped pinv recover joint velocities accurately."""
    V = kin.forward_velocity_kinematics(q, q_dot)
    q_dot_dls = kin.inverse_velocity_kinematics(q, V, damping=0.01)
    q_dot_pinv = kin.inverse_velocity_kinematics(q, V, damping=0.0)
    assert np.linalg.norm(q_dot_dls - np.asarray(q_dot)) < expected_dls_bound
    assert np.linalg.norm(q_dot_pinv - np.asarray(q_dot)) < 1e-14


def test_normalize_angle_range() -> None:
    """Angles wrap into [-pi, pi]."""
    for raw in [-5.0 * math.pi, -math.pi, 0.0, math.pi, 3.25 * math.pi]:
        wrapped = kin.normalize_angle(raw)
        assert -math.pi <= wrapped <= math.pi
        assert math.isclose(math.sin(raw), math.sin(wrapped), abs_tol=1e-12)
        assert math.isclose(math.cos(raw), math.cos(wrapped), abs_tol=1e-12)


def test_linear_task_space_trajectory() -> None:
    """Cartesian trajectory has exact endpoints and uniform step increments."""
    start = [0.1, -0.2, 0.25]
    end = [-0.15, 0.25, 0.35]
    times, pts_list = tp.get_task_space_trajectory(start, end, total_time=2.0, dt=0.1)
    pts = np.asarray(pts_list)
    assert len(times) == 21
    assert pts.shape == (21, 3)
    assert np.allclose(pts[0], start, atol=1e-12)
    assert np.allclose(pts[-1], end, atol=1e-12)
    diffs = np.diff(pts, axis=0)
    assert np.allclose(diffs, diffs[0], atol=1e-12)


def test_circular_trajectory_geometry() -> None:
    """Circular trajectory maintains fixed z and radius, closing cleanly at 2*pi."""
    center = [-0.20, 0.0, 0.20]
    radius = 0.04
    times, pts_list = tp.get_circular_trajectory(center, radius, total_time=4.0, dt=0.1)
    pts = np.asarray(pts_list)
    assert len(times) == 41
    xy_dist = np.linalg.norm(pts[:, :2] - np.array(center[:2]), axis=1)
    assert np.allclose(xy_dist, radius, atol=1e-12)
    assert np.allclose(pts[:, 2], center[2], atol=1e-12)
    assert np.allclose(pts[0], pts[-1], atol=1e-12)


def test_cubic_joint_trajectory_boundary_conditions() -> None:
    """Cubic polynomial matches endpoints with zero initial and final velocity."""
    q0 = [0.0, 0.2, -0.1, 0.3]
    qf = [0.5, -0.3, 0.4, -0.2]
    tf = 2.0
    dt = 0.01
    times, traj_list = tp.get_joint_space_trajectory(q0, qf, total_time=tf, dt=dt)
    traj = np.asarray(traj_list)
    assert len(times) == 201
    assert np.allclose(traj[0], q0, atol=1e-12)
    assert np.allclose(traj[-1], qf, atol=1e-12)
    v_start = (traj[1] - traj[0]) / dt
    v_end = (traj[-1] - traj[-2]) / dt
    assert np.all(np.abs(v_start) < 0.02)
    assert np.all(np.abs(v_end) < 0.02)


@pytest.mark.parametrize(
    ("filename", "expected_rows"),
    [
        ("trajectory_data_combined_1_square1.csv", 103),
        ("trajectory_data_combined_2_square2.csv", 103),
        ("trajectory_data_combined_3_square2_rev.csv", 103),
        ("trajectory_data_combined_4_transition.csv", 41),
        ("trajectory_data_combined_5_circle.csv", 101),
    ],
)
def test_hardware_csv_waypoints_valid(filename: str, expected_rows: int) -> None:
    """Every trajectory CSV exists, matches expected waypoint count, and has 4 finite angles."""
    send_traj = _load_send_trajectory()
    rows = send_traj.load_trajectory_csv(TRAJECTORIES_DIR / filename)
    assert len(rows) == expected_rows
    arr = np.asarray(rows, dtype=float)
    assert arr.shape == (expected_rows, 4)
    assert np.all(np.isfinite(arr))
    assert np.all(np.abs(arr) <= 180.0)


def test_send_trajectory_header_and_dry_run(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Header rows are skipped when present and CLI --dry-run prints raw CSV wire commands."""
    send_traj = _load_send_trajectory()
    sample = tmp_path / "headered.csv"
    sample.write_text("q1,q2,q3,q4\n10.0,20.0,-30.0,40.0\n", encoding="utf-8")
    assert send_traj.load_trajectory_csv(sample) == [[10.0, 20.0, -30.0, 40.0]]
    assert send_traj.main(["--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "J:" not in out
    assert "  2. 33.66,-4.69,57.08,-89.94" in out
