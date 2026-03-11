#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from forward_kinematics import forward_kinematics_func # This now imports the modified version
import numpy as np

latest_ee_pose = None

def ee_pose_callback(msg):
    global latest_ee_pose
    latest_ee_pose = msg.pose.position

def main():
    rospy.init_node('fk_validation_node')

    # Create Publishers for all 5 joints
    pub_joint1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
    pub_joint2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
    pub_joint3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
    pub_joint4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
    pub_joint5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)

    rospy.loginfo("Waiting for /end_effector_pose message...")
    try:
        rospy.wait_for_message('/end_effector_pose', PoseStamped, timeout=10.0)
        rospy.loginfo("/end_effector_pose is active.")
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /end_effector_pose.")
        return

    rospy.Subscriber('/end_effector_pose', PoseStamped, ee_pose_callback)
    rospy.sleep(1.0)

    # --- Test Case ---
    # Define angles for the new arm configuration (q4 is now fixed)
    q1, q2, q3, q5 = 0, 0, 0, 0
    q4 = 0.0 # Disregarded joint is set to a constant value

    # Publish to all 5 joints
    pub_joint1.publish(q1)
    pub_joint2.publish(q2)
    pub_joint3.publish(q3)
    pub_joint4.publish(q4)
    pub_joint5.publish(q5)
    rospy.loginfo("Published joint angles: q1-q3={}, q4={}, q5={}".format([q1, q2, q3], q4, q5))

    rospy.sleep(3.0)

    # Calculate the expected position using the new set of variable joints
    calculated_position = forward_kinematics_func(q1, q2, q3, q5)
    
    # Print the comparison
    print("\n--- Milestone 2 Validation (New 4-DOF Arm: q1,q2,q3,q5) ---")
    print(f"Calculated (Predicted) Position: [x={calculated_position[0]:.4f}, y={calculated_position[1]:.4f}, z={calculated_position[2]:.4f}]")
    if latest_ee_pose:
        print(f"Actual (from Gazebo /end_effector_pose): [x={latest_ee_pose.x:.4f}, y={latest_ee_pose.y:.4f}, z={latest_ee_pose.z:.4f}]")
        
        error = np.array(calculated_position) - np.array([latest_ee_pose.x, latest_ee_pose.y, latest_ee_pose.z])
        error_magnitude = np.linalg.norm(error)
        print(f"\nPosition Error Magnitude: {error_magnitude:.6f}")
    else:
        print("Could not retrieve actual position.")
    print("----------------------------------------------------------\n")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
