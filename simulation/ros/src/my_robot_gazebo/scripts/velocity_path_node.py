#!/usr/bin/env python3
"""
velocity_path_node.py

ROS helper node that mirrors run_forward_velocity_test for a single command:
  - Provide one desired cartesian pose (X_DES) and a velocity vector (V_DES).
  - The node solves IK to reach X_DES, converts V_DES to joint velocities via
    inverse_velocity_kinematics, and then drives Gazebo with that constant q_dot
    for TEST_DURATION seconds while logging the measured EE velocity.
"""

from __future__ import annotations

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from kinematics import (
    inverse_kinematics_func,
    inverse_velocity_kinematics,
    forward_velocity_kinematics,
)

# ---------------------------------------------------------------------------
# User-editable inputs
# ---------------------------------------------------------------------------

X_DES = np.array([0.0000, 0.0000, 0.3])  # desired EE pose [x, y, z]
V_DES = np.array([0.000, 0.000, -0.100, 0.0, 0.0, 0.0])  # spatial velocity [m/s, rad/s]
Q_INITIAL = np.array([0.0, 0.0, 0.0, 0.0])  # IK initial guess

DT_CMD = 0.01        # command period (s)
TEST_DURATION = 3.0  # time to run constant q_dot (s)
SETTLE_TIME = 1.0    # hold start pose before commanding velocity (s)
LOG_EVERY = 10       # log cadence in command steps

# Globals updated from /end_effector_pose
ee_pos = None    # np.array([x,y,z])
ee_stamp = None  # float seconds


def ee_cb(msg):
    """Keep track of the latest end-effector pose from Gazebo."""
    global ee_pos, ee_stamp
    ee_pos = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        dtype=float,
    )
    ee_stamp = msg.header.stamp.to_sec()


def publish_joint_commands(pubs, q):
    """Publish joint angles in the order [Joint_1, Joint_2, Joint_3, Joint_5]."""
    pubs[0].publish(Float64(float(q[0])))
    pubs[1].publish(Float64(float(q[1])))
    pubs[2].publish(Float64(float(q[2])))
    pubs[3].publish(Float64(float(q[3])))


def _as_spatial(vec):
    """Ensure the provided vector has 6 spatial components."""
    vec = np.asarray(vec, dtype=float).ravel()
    if vec.size == 3:
        return np.hstack((vec, np.zeros(3)))
    if vec.size != 6:
        raise ValueError(
            f"x_dot entries must have 3 or 6 numbers, received {vec.size}"
        )
    return vec


def run_forward_velocity_test(pubs, q_start, q_dot, duration, dt_cmd, settle_time):
    """
    Publish joints at a constant velocity and compare theoretical vs measured EE velocity.
    Mirrors the helper added to ik_validation_node.
    """
    global ee_pos, ee_stamp

    q_dot = np.asarray(q_dot, dtype=float).ravel()
    assert q_dot.size == 4, "q_dot must be length 4"

    rate = rospy.Rate(1.0 / dt_cmd)
    q = np.array(q_start, dtype=float).copy()

    rospy.loginfo("Forward velocity test: q_start=%s", q.round(4).tolist())
    rospy.loginfo("Commanding q_dot=%s rad/s", q_dot.round(4).tolist())

    # Hold starting pose to let Gazebo settle
    rospy.loginfo("Holding start pose for %.2f s", settle_time)
    for _ in range(int(settle_time / dt_cmd)):
        publish_joint_commands(pubs, q)
        rate.sleep()

    num_steps = int(max(duration, 0.0) / dt_cmd)
    prev_ee = None
    prev_t = None

    rospy.loginfo("Running velocity segment for %.2f s (%d steps)", duration, num_steps)

    for k in range(num_steps):
        if rospy.is_shutdown():
            break

        q = q + q_dot * dt_cmd
        publish_joint_commands(pubs, q)

        # Expectation from Jacobian-based forward velocity
        V_theory = forward_velocity_kinematics(q, q_dot)
        v_theory_lin = V_theory[:3]

        if ee_pos is not None and ee_stamp is not None:
            if prev_ee is not None and prev_t is not None:
                dt_pose = ee_stamp - prev_t
                if dt_pose > 0.0:
                    v_numeric = (ee_pos - prev_ee) / dt_pose
                    err = v_theory_lin - v_numeric
                    if LOG_EVERY > 0 and k % LOG_EVERY == 0:
                        rospy.loginfo(
                            "[Step %3d] q=%s | EE_pos=[%.4f, %.4f, %.4f]",
                            k,
                            q.round(4).tolist(),
                            ee_pos[0],
                            ee_pos[1],
                            ee_pos[2],
                        )
                        rospy.loginfo(
                            "           V_theory_lin=[%.4f, %.4f, %.4f] m/s | "
                            "V_numeric=[%.4f, %.4f, %.4f] m/s | err=%.6f",
                            v_theory_lin[0],
                            v_theory_lin[1],
                            v_theory_lin[2],
                            v_numeric[0],
                            v_numeric[1],
                            v_numeric[2],
                            np.linalg.norm(err),
                        )

            prev_ee = ee_pos.copy()
            prev_t = ee_stamp

        rate.sleep()

    rospy.loginfo("Segment complete.")
    return q


def main():
    rospy.init_node("velocity_path_node")

    pubs = [
        rospy.Publisher('/Joint_1/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_2/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_3/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_5/command', Float64, queue_size=10),
    ]

    rospy.Subscriber('/end_effector_pose', PoseStamped, ee_cb, queue_size=1)
    rospy.loginfo("Waiting for /end_effector_pose...")
    rospy.wait_for_message('/end_effector_pose', PoseStamped, timeout=10.0)
    rospy.loginfo("/end_effector_pose is active.")

    x_des = np.asarray(X_DES, dtype=float).ravel()
    v_des = _as_spatial(V_DES)

    rospy.loginfo("Solving IK for X_DES=%s", x_des.tolist())
    q_start = inverse_kinematics_func(np.array(Q_INITIAL, dtype=float), x_des)
    rospy.loginfo("IK solution q_start=%s", q_start.round(4).tolist())

    q_dot = inverse_velocity_kinematics(q_start, v_des)
    rospy.loginfo("Derived q_dot=%s", q_dot.round(4).tolist())

    run_forward_velocity_test(
        pubs,
        q_start,
        q_dot,
        duration=TEST_DURATION,
        dt_cmd=DT_CMD,
        settle_time=SETTLE_TIME,
    )

    rospy.loginfo("Velocity test complete.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
