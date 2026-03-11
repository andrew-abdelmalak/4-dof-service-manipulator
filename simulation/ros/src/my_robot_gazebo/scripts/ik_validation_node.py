#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
import numpy as np

from kinematics import (
    jacobian_matrix,
    inverse_kinematics_func,      # uses your MS02 FK internally
    forward_velocity_kinematics,
    inverse_velocity_kinematics
)

# Latest end-effector state from Gazebo
ee_pos = None    # np.array([x, y, z])
ee_stamp = None  # float: time in seconds


def ee_cb(msg):
    """Callback for /end_effector_pose."""
    global ee_pos, ee_stamp
    ee_pos = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
        dtype=float
    )
    ee_stamp = msg.header.stamp.to_sec()


def publish_joint_commands(pubs, q):
    """
    Publish joint angles to Gazebo.
    q = [q1, q2, q3, q4_math]  where q4_math corresponds to Joint_5.
    """
    pubs[0].publish(Float64(float(q[0])))  # Joint_1
    pubs[1].publish(Float64(float(q[1])))  # Joint_2
    pubs[2].publish(Float64(float(q[2])))  # Joint_3
    pubs[3].publish(Float64(float(q[3])))  # Joint_5  (wrist)


def make_layout(rows, cols):
    """Create MultiArray layout for a (rows x cols) matrix."""
    arr = Float64MultiArray()
    layout = arr.layout
    layout.dim = [
        MultiArrayDimension(label="rows", size=rows, stride=rows * cols),
        MultiArrayDimension(label="cols", size=cols, stride=cols),
    ]
    layout.data_offset = 0
    return layout


def run_forward_velocity_test(
    pubs,
    q_start,
    q_dot_override=None,
    duration=3.0,
    dt_cmd=0.01,
    settle_time=1.0,
    log_every=10,
):
    """
    Forward velocity test:
      - Start from q_start
      - Apply constant joint velocity q_dot
      - Compare theoretical Cartesian velocity V = J(q) q_dot
        with numeric velocity estimated from Gazebo EE positions.
    """
    global ee_pos, ee_stamp

    # Choose joint velocities [rad/s]
    if q_dot_override is None:
        # Default motion: only joint 1 moving
        q_dot = np.array([0.00, 0.0, 0.1, 0.0], dtype=float)
    else:
        q_dot = np.asarray(q_dot_override, dtype=float).ravel()
        assert q_dot.size == 4, "q_dot_override must be length 4"

    # Integration step for joint commands
    rate = rospy.Rate(1.0 / dt_cmd)

    q = np.array(q_start, dtype=float).copy()

    rospy.loginfo("Forward velocity test: starting from q = %s", q.round(4).tolist())
    rospy.loginfo("Using joint velocities q_dot = %s [rad/s]", q_dot.round(4).tolist())

    # Let the robot settle at the starting configuration
    rospy.loginfo("Sending q_start to Gazebo and waiting %.2f s...", settle_time)
    for _ in range(int(settle_time / dt_cmd)):
        publish_joint_commands(pubs, q)
        rate.sleep()

    prev_ee = None
    prev_t = None
    num_steps = int(max(duration, 0.0) / dt_cmd)

    rospy.loginfo("Starting forward velocity loop (%d steps, dt_cmd=%.3f s)...",
                  num_steps, dt_cmd)

    for k in range(num_steps):
        if rospy.is_shutdown():
            break

        # Integrate joint angles: q(t+dt) = q(t) + q_dot * dt_cmd
        q = q + q_dot * dt_cmd

        # Send to Gazebo
        publish_joint_commands(pubs, q)

        # Theoretical spatial velocity from Jacobian
        V_theory = forward_velocity_kinematics(q, q_dot)  # [vx, vy, vz, wx, wy, wz]
        v_theory_lin = V_theory[:3]

        # Numeric linear velocity from Gazebo poses (finite difference using real dt)
        if ee_pos is not None and ee_stamp is not None:
            if prev_ee is not None and prev_t is not None:
                dt_pose = ee_stamp - prev_t
                if dt_pose > 0.0:
                    v_numeric = (ee_pos - prev_ee) / dt_pose

                    vel_err = v_theory_lin - v_numeric
                    err_norm = np.linalg.norm(vel_err)

                    # Log at the requested cadence to avoid spam
                    if log_every > 0 and k % log_every == 0:
                        rospy.loginfo(
                            "[Step %3d] q = %s",
                            k, q.round(4).tolist()
                        )
                        rospy.loginfo(
                            "           EE_pos = [%.4f, %.4f, %.4f]",
                            ee_pos[0], ee_pos[1], ee_pos[2]
                        )
                        rospy.loginfo(
                            "           V_theory_lin = [%.4f, %.4f, %.4f] m/s | "
                            "V_numeric = [%.4f, %.4f, %.4f] m/s | err_norm = %.6f",
                            v_theory_lin[0], v_theory_lin[1], v_theory_lin[2],
                            v_numeric[0], v_numeric[1], v_numeric[2],
                            err_norm
                        )

            prev_ee = ee_pos.copy()
            prev_t = ee_stamp

        rate.sleep()

    rospy.loginfo("Forward velocity test finished.")
    return q


def main():
    rospy.init_node("ik_validation_node")

    # Publishers for your 4 active joints (q1, q2, q3, q4_math->Joint_5)
    pubs = [
        rospy.Publisher('/Joint_1/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_2/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_3/command', Float64, queue_size=10),
        rospy.Publisher('/Joint_5/command', Float64, queue_size=10),
    ]

    # Jacobian publisher: 6x4 matrix, row-major
    jac_pub = rospy.Publisher('/jacobian', Float64MultiArray, queue_size=10)

    # Subscribe to end-effector pose
    rospy.Subscriber('/end_effector_pose', PoseStamped, ee_cb, queue_size=1)
    rospy.loginfo("Waiting for /end_effector_pose...")
    rospy.wait_for_message('/end_effector_pose', PoseStamped, timeout=10.0)
    rospy.loginfo("/end_effector_pose is active.")

    # -------- Inverse Position Kinematics validation --------
    # Desired EE position (you can change this if you want)
    X_des = np.array([0.0000, 0.0000, 0.4255], dtype=float)
    q0 = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)  # initial guess

    rospy.loginfo("X_des: %s", X_des.tolist())
    rospy.loginfo("q0: %s", q0.tolist())

    q_des = inverse_kinematics_func(q0, X_des)
    rospy.loginfo("q_des (IK solution): %s", np.array(q_des).round(4).tolist())

    # Send joints to Gazebo
    publish_joint_commands(pubs, q_des)
    rospy.sleep(2.5)  # let Gazebo update

    # Compare with actual Gazebo EE pose if available
    if ee_pos is not None:
        X_meas = ee_pos.copy()
        err = X_des - X_meas
        rospy.loginfo("Gazebo EE pose: X=%.4f, Y=%.4f, Z=%.4f",
                      X_meas[0], X_meas[1], X_meas[2])
        rospy.loginfo("||X_des - X_meas|| = %.6f m", np.linalg.norm(err))
    else:
        rospy.logwarn("No /end_effector_pose received after sending joints.")

    # -------- Forward velocity test in Gazebo --------
    run_forward_velocity_test(pubs, q_des)

    # -------- Continuous Jacobian publishing at q_des --------
    rospy.loginfo("Starting continuous /jacobian publishing at 10 Hz...")
    layout = make_layout(6, 4)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        J = jacobian_matrix(q_des)  # 6x4
        msg = Float64MultiArray()
        msg.layout = layout
        msg.data = J.flatten().tolist()  # row-major
        jac_pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as e:
        rospy.logerr(e)
