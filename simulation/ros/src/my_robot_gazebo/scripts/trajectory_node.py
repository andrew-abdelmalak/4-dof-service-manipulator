#!/usr/bin/env python3
import rospy
import numpy as np
import sys
import csv
import os
from std_msgs.msg import Float64
from trajectory_planner import get_task_space_trajectory, get_circular_trajectory
from kinematics import inverse_kinematics_func

def normalize_angle(angle):
    """Wraps an angle to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))

def publish_joints(publishers, q_values):
    """Publishes angles to Gazebo after normalizing them."""
    for i, pub in enumerate(publishers):
        norm_q = normalize_angle(q_values[i])
        pub.publish(Float64(norm_q))

def save_to_csv(writer, q_sol):
    """
    Helper function to write data to CSV.
    Logs ONLY: Q_deg(1-4)
    """
    # Convert radians to degrees
    q_deg = [np.degrees(q) for q in q_sol]
    
    # Write only the degrees to the CSV
    writer.writerow(q_deg)

def delay_with_logging(duration, dt, pubs, q_current, csv_writer):
    """
    Holds the current position for 'duration' seconds while continuing to publish and log to CSV.
    """
    rate = rospy.Rate(1/dt)
    num_steps = int(duration / dt)
    
    rospy.loginfo(f"=== Holding position for {duration} second(s) ===")
    for _ in range(num_steps):
        if rospy.is_shutdown(): break
        
        # Keep publishing current position
        publish_joints(pubs, q_current)
        
        # Continue logging to CSV
        save_to_csv(csv_writer, q_current)
        
        rate.sleep()

def run_joint_interpolation(q_start, q_end, duration, dt, pubs, csv_writer):
    """
    Executes a linear trajectory in JOINT SPACE from q_start to q_end.
    Useful for safely reconfiguring the robot between segments.
    """
    rate = rospy.Rate(1/dt)
    steps = int(duration / dt)
    
    q_start_arr = np.array(q_start)
    q_end_arr = np.array(q_end)
    
    rospy.loginfo(f"Starting Linear Joint Transition: {np.round(q_start, 2)} -> {np.round(q_end, 2)}")
    
    for i in range(steps + 1):
        if rospy.is_shutdown(): break
        
        # Linear Interpolation (s goes from 0.0 to 1.0)
        s = i / float(steps)
        q_curr = (1 - s) * q_start_arr + s * q_end_arr
        
        publish_joints(pubs, q_curr)
        save_to_csv(csv_writer, q_curr)
        
        if i % 10 == 0:
             rospy.loginfo(f"Transition T: {i*dt:.1f}/{duration} | q: {np.round(q_curr, 2)}")

        rate.sleep()
        
    return q_end_arr

def run_linear_segments(waypoints, segment_duration, dt, pubs, q_prev, csv_writer):
    """
    Executes a sequence of linear segments defined by waypoints.
    Logs data to CSV at every step.
    Returns final joint configuration.
    """
    rospy.loginfo(f"Moving to Start Point: {waypoints[0]}...")
    
    # -- Step 1: Move to Start Point --
    # Solve IK for the first point using q_prev as guess
    q_start = inverse_kinematics_func(q_prev, waypoints[0])
    q_start = [normalize_angle(q) for q in q_start]
    
    # Publish and wait for robot to reach start
    publish_joints(pubs, q_start)
    rospy.sleep(4.0) 
    
    q_prev = q_start
    rate = rospy.Rate(1/dt)

    # -- Step 2: Loop through segments --
    for i in range(len(waypoints) - 1):
        start_pt = waypoints[i]
        end_pt = waypoints[i+1]
        
        rospy.loginfo(f"--- Starting Segment {i+1}: {np.round(start_pt, 2)} -> {np.round(end_pt, 2)} ---")
        
        # Generate Linear Trajectory for this segment
        times, points = get_task_space_trajectory(start_pt, end_pt, segment_duration, dt)
        
        for t, point in zip(times, points):
            if rospy.is_shutdown(): break
            
            # Solve IK
            q_sol = inverse_kinematics_func(q_prev, point)
            
            # Safety Check
            if np.any(np.abs(q_sol) > 100.0):
                rospy.logerr("Unsafe joint values detected! Stopping.")
                return q_prev

            # Publish
            publish_joints(pubs, q_sol)
            
            # 1. Terminal Output (Detailed)
            rospy.loginfo(f"Seg {i+1} | T: {t:.1f} | Tgt: {np.round(point, 3)} | q: {np.round(q_sol, 2)}")
            
            # 2. Save to CSV (Degrees Only)
            save_to_csv(csv_writer, q_sol)
            
            q_prev = q_sol 
            rate.sleep()
        
        # Short pause at each corner/waypoint
        rospy.sleep(0.5)
    
    return q_prev

def mode_square1(segment_duration, dt, pubs, q_start, csv_writer, waypoints_config):
    """Execute Square 1 trajectory: A -> B -> C -> D"""
    rospy.loginfo("Mode Square 1: Right Side (A->B->C->D)")
    pA, pB, pC, pD = waypoints_config['pA'], waypoints_config['pB'], waypoints_config['pC'], waypoints_config['pD']
    waypoints = [pA, pB, pC, pD]
    return run_linear_segments(waypoints, segment_duration, dt, pubs, q_start, csv_writer)

def mode_square2(segment_duration, dt, pubs, q_start, csv_writer, waypoints_config):
    """Execute Square 2 trajectory: D -> C -> E -> F"""
    rospy.loginfo("Mode Square 2: Left Side (D->C->E->F)")
    pC, pD, pE, pF = waypoints_config['pC'], waypoints_config['pD'], waypoints_config['pE'], waypoints_config['pF']
    waypoints = [pD, pC, pE, pF]
    return run_linear_segments(waypoints, segment_duration, dt, pubs, q_start, csv_writer)

def mode_square2_reversed(segment_duration, dt, pubs, q_start, csv_writer, waypoints_config):
    """Execute Square 2 Reversed trajectory: F -> E -> C -> D"""
    rospy.loginfo("Mode Square 2 Reversed: (F->E->C->D)")
    pC, pD, pE, pF = waypoints_config['pC'], waypoints_config['pD'], waypoints_config['pE'], waypoints_config['pF']
    waypoints = [pF, pE, pC, pD]
    return run_linear_segments(waypoints, segment_duration, dt, pubs, q_start, csv_writer)

def mode_circle(circle_config, dt, pubs, q_start, csv_writer):
    """Execute circular trajectory"""
    circle_center = circle_config['center']
    circle_radius = circle_config['radius']
    circle_duration = circle_config['duration']
    
    # Calculate Circle Start (Rightmost point)
    circle_start_pos = [circle_center[0] + circle_radius, circle_center[1], circle_center[2]]

    rospy.loginfo(f"Moving to Circle Start: {circle_start_pos}...")
    q_circle_start = inverse_kinematics_func(q_start, circle_start_pos)
    publish_joints(pubs, q_circle_start)
    rospy.sleep(4.0) 

    rospy.loginfo("Starting Circular Motion...")
    times, points = get_circular_trajectory(circle_center, circle_radius, circle_duration, dt)
    
    q_prev = q_circle_start 
    rate = rospy.Rate(1/dt)

    for t, point in zip(times, points):
        if rospy.is_shutdown(): break
        q_sol = inverse_kinematics_func(q_prev, point)
        publish_joints(pubs, q_sol)
        
        # 1. Terminal Output (Detailed)
        rospy.loginfo(f"T: {t:.1f} | Tgt: {np.round(point,3)} | q: {np.round(q_sol, 2)}")
        
        # 2. Save to CSV (Degrees Only)
        save_to_csv(csv_writer, q_sol)
        
        q_prev = q_sol 
        rate.sleep()
    
    return q_prev

def main():
    rospy.init_node('trajectory_node')

    # Publishers [Joint1, Joint2, Joint3, Joint5]
    pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
    pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
    pub5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)
    
    pubs = [pub1, pub2, pub3, pub5]

    # --- TRAJECTORY CONFIGURATION ---
    # Waypoints configuration
    waypoints_config = {
        'pA': [0.15, 0.10, 0.15],  # Start Right Low
        'pB': [0.15, 0.10, 0.20],  # Right High
        'pC': [0.15, 0.0,  0.20],  # Middle High (Center)
        'pD': [0.15, 0.0,  0.15],  # Middle Low (Center)
        'pE': [0.15, -0.10, 0.20], # Left High
        'pF': [0.15, -0.10, 0.15]  # Left Low
    }
    
    # Circle configuration
    circle_config = {
        'center': [0.15, 0.0, 0.15],
        'radius': 0.03,
        'duration': 10.0
    }
    
    segment_duration = 3.0 # Seconds per line
    dt = 0.1       

    # --- CHOOSE MODE ---
    mode = rospy.get_param("~mode", "circle")
    home_dir = os.path.expanduser('~')
    header = ['Q1_deg', 'Q2_deg', 'Q3_deg', 'Q4_deg']
    
    rospy.loginfo(f"--- Starting Milestone 4: {mode.upper()} Trajectory ---")
    
    # 1. Move to Home Position (Common Start)
    q_home = [0, 0, 0, 0]
    publish_joints(pubs, q_home)
    rospy.sleep(2.0)
    
    if mode == 'combined':
        rospy.loginfo("Mode COMBINED: Splitting data into 5 CSV files.")
        q_current = q_home

        # === PART 1: SQUARE 1 ===
        csv_name = "trajectory_data_combined_1_square1.csv"
        csv_path = os.path.join(home_dir, csv_name)
        rospy.loginfo(f"=== PART 1: SQUARE 1 (Saving to {csv_name}) ===")
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            q_current = mode_square1(segment_duration, dt, pubs, q_current, writer, waypoints_config)
            delay_with_logging(1.0, dt, pubs, q_current, writer)

        # === PART 2: SQUARE 2 ===
        csv_name = "trajectory_data_combined_2_square2.csv"
        csv_path = os.path.join(home_dir, csv_name)
        rospy.loginfo(f"=== PART 2: SQUARE 2 (Saving to {csv_name}) ===")
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            q_current = mode_square2(segment_duration, dt, pubs, q_current, writer, waypoints_config)
            delay_with_logging(1.0, dt, pubs, q_current, writer)

        # === PART 3: SQUARE 2 REVERSED ===
        csv_name = "trajectory_data_combined_3_square2_rev.csv"
        csv_path = os.path.join(home_dir, csv_name)
        rospy.loginfo(f"=== PART 3: SQUARE 2 REVERSED (Saving to {csv_name}) ===")
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            q_current = mode_square2_reversed(segment_duration, dt, pubs, q_current, writer, waypoints_config)
            delay_with_logging(1.0, dt, pubs, q_current, writer)

        # === PART 4: TRANSITION TO SAFE CONFIGURATION ===
        csv_name = "trajectory_data_combined_4_transition.csv"
        csv_path = os.path.join(home_dir, csv_name)
        rospy.loginfo(f"=== PART 4: TRANSITION (Saving to {csv_name}) ===")
        
        # Configuration that works fine for standalone circle
        q_safe = [0.0, -0.08, 1.0, -1.57] 
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            # Execute Linear Joint Trajectory to Safe Point
            q_current = run_joint_interpolation(q_current, q_safe, segment_duration, dt, pubs, writer)
            delay_with_logging(1.0, dt, pubs, q_current, writer)

        # === PART 5: CIRCLE ===
        csv_name = "trajectory_data_combined_5_circle.csv"
        csv_path = os.path.join(home_dir, csv_name)
        rospy.loginfo(f"=== PART 5: CIRCLE (Saving to {csv_name}) ===")
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            q_current = mode_circle(circle_config, dt, pubs, q_current, writer)
            
    else:
        # SINGLE MODE EXECUTION
        csv_filename = f"trajectory_data_{mode}.csv"
        csv_path = os.path.join(home_dir, csv_filename)
        rospy.loginfo(f"--- Data will be saved to: {csv_path} ---")
        
        with open(csv_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(header)

            if mode == 'square1':
                mode_square1(segment_duration, dt, pubs, q_home, csv_writer, waypoints_config)

            elif mode == 'square2':
                mode_square2(segment_duration, dt, pubs, q_home, csv_writer, waypoints_config)

            elif mode == 'circle':
                mode_circle(circle_config, dt, pubs, q_home, csv_writer)
            
            else:
                rospy.logerr(f"Unknown mode '{mode}'. Available: circle, square1, square2, combined")

    rospy.loginfo("Trajectory Completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
