#!/usr/bin/env python3
import numpy as np

def get_task_space_trajectory(X_start, X_end, total_time, dt):
    """
    Generates a Linear Task-Space Trajectory.
    Formula: X(t) = X0 + alpha * t
    """
    steps = int(total_time / dt)
    trajectory_points = []
    time_steps = []

    X_start = np.array(X_start)
    X_end = np.array(X_end)

    # Velocity vector
    velocity_vector = (X_end - X_start) / total_time

    for i in range(steps + 1):
        t = i * dt
        current_pos = X_start + velocity_vector * t
        trajectory_points.append(current_pos)
        time_steps.append(t)

    return time_steps, trajectory_points

def get_circular_trajectory(center, radius, total_time, dt):
    """
    Generates a Circular Task-Space Trajectory in the XY plane (rotating around Z).
    """
    steps = int(total_time / dt)
    trajectory_points = []
    time_steps = []
    
    center = np.array(center)
    # Extract z height (constant for a flat circle)
    z_height = center[2]

    for i in range(steps + 1):
        t = i * dt
        
        # Calculate angle theta based on time (0 to 2*pi)
        theta = (2 * np.pi / total_time) * t
        
        # Parametric Circle Equations:
        # x = cx + r * cos(theta)
        # y = cy + r * sin(theta)
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        z = z_height # Constant Z

        trajectory_points.append(np.array([x, y, z]))
        time_steps.append(t)

    return time_steps, trajectory_points

def get_cubic_coeffs(q_start, q_end, tf):
    """ Calculates coefficients for Cubic Polynomial: q(t) = c0 + c1*t + c2*t^2 + c3*t^3 """
    c0 = q_start
    c1 = 0
    c2 = 3 * (q_end - q_start) / (tf ** 2)
    c3 = -2 * (q_end - q_start) / (tf ** 3)
    return c0, c1, c2, c3

def get_joint_space_trajectory(q_start, q_end, total_time, dt):
    """ Generates a Joint-Space Cubic Trajectory for all joints. """
    steps = int(total_time / dt)
    num_joints = len(q_start)
    
    # Calculate coefficients for EACH joint
    coeffs = []
    for j in range(num_joints):
        coeffs.append(get_cubic_coeffs(q_start[j], q_end[j], total_time))

    trajectory_qs = []
    time_steps = []

    for i in range(steps + 1):
        t = i * dt
        current_q = []
        
        for j in range(num_joints):
            c0, c1, c2, c3 = coeffs[j]
            q_val = c0 + (c1 * t) + (c2 * t**2) + (c3 * t**3)
            current_q.append(q_val)
            
        trajectory_qs.append(np.array(current_q))
        time_steps.append(t)

    return time_steps, trajectory_qs
