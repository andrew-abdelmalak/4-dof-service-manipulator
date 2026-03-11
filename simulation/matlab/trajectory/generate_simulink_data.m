% generate_simulink_data.m
% Generates Circular Trajectory Joint Angles and converts them 
% to TimeSeries objects for Simulink/Simscape.

clear; clc;

% --- 1. CONFIGURATION (Matches Python/Milestone 4) ---
circle_center = [0.15; 0.0; 0.15];
circle_radius = 0.03;
total_time = 10.0; % Duration of the circle
dt = 0.01;         % Small time step for smooth Simulink motion

% Robot Link Lengths (Needed for IK context if not hardcoded in functions)
% l1=0.04355, l2=0.140, l3=0.1324, l4=0.01213

% --- 2. GENERATE TRAJECTORY POINTS ---
% Calculate Circle Start Position
start_pos = [circle_center(1) + circle_radius; circle_center(2); circle_center(3)];

% Generate the Cartesian path
[time_steps, points_3d] = get_circular_trajectory(circle_center, circle_radius, total_time, dt);
num_steps = length(time_steps);

% --- 3. SOLVE INVERSE KINEMATICS ---
fprintf('Calculating Inverse Kinematics for %d points...\n', num_steps);

% Pre-allocate joint arrays (4 joints x N steps)
q_data = zeros(4, num_steps);

% Initial IK guess (Home Position)
q_prev = [0; 0; 0; 0];

% First move: Solve for the start of the circle to avoid a "jump"
q_start = inverse_kinematics_func(q_prev, start_pos);
q_prev = q_start;

for i = 1:num_steps
    target_point = points_3d(:, i);
    
    % Solve IK for current point
    q_sol = inverse_kinematics_func(q_prev, target_point);
    
    % Store result
    q_data(:, i) = q_sol;
    
    % Update previous guess for continuity
    q_prev = q_sol;
end

% --- 4. CREATE TIMESERIES FOR SIMULINK ---
% Create timeseries objects. 
% Simulink "From Workspace" blocks look for these variable names.

q1_ts = timeseries(q_data(1, :)', time_steps);
q2_ts = timeseries(q_data(2, :)', time_steps);
q3_ts = timeseries(q_data(3, :)', time_steps);
q4_ts = timeseries(q_data(4, :)', time_steps);

% Also create a pure Cartesian timeseries if you want to compare X,Y,Z errors
ref_pos_ts = timeseries(points_3d', time_steps);

% --- 5. SAVE/REPORT ---
fprintf('Done! Variables q1_ts, q2_ts, q3_ts, q4_ts are ready in the Workspace.\n');
fprintf('You can now run your Simulink model.\n');

% Optional: Save to a .mat file for later loading
save('trajectory_data.mat', 'q1_ts', 'q2_ts', 'q3_ts', 'q4_ts', 'ref_pos_ts');