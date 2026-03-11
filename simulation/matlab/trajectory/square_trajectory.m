% generate_simulink_data_v5.m
% Generates a PERFECT SQUARE trajectory (5cm x 5cm)
% A -> B (Up), B -> C (Side), C -> D (Down)
% Includes pauses at corners for sharp definition.



% --- 1. CONFIGURATION ---
MODE = 'square'; 
segment_duration = 5.0; 
dt = 0.1; 

% --- 2. DEFINE PERFECT SQUARE COORDINATES ---
% We use a side length of 5cm (0.05m) for both height and width.
side_length = 0.05;

% Fixed X-plane (Front of robot)
X_fixed = 0.15;

% Z-coordinates (Height)
Z_low   = 0.15;
Z_high  = Z_low + side_length; % 0.20

% Y-coordinates (Side to Side)
% We center it slightly so it's easy to reach
Y_left  = 0.05; 
Y_right = Y_left - side_length; % 0.00

% Define Points
pA = [X_fixed; Y_left;  Z_low];  % Start
pB = [X_fixed; Y_left;  Z_high]; % Top-Left
pC = [X_fixed; Y_right; Z_high]; % Top-Right
pD = [X_fixed; Y_right; Z_low];  % Bottom-Right

% --- 3. GENERATE TRAJECTORY POINTS ---
points_3d = [];
time_steps = [];

if strcmp(MODE, 'square')
    fprintf('Generating PERFECT SQUARE Trajectory (5cm x 5cm)...\n');
    waypoints = [pA, pB, pC, pD]; 
    
    current_time = 0;
    
    % Loop through segments
    for i = 1:(size(waypoints, 2) - 1)
        start_pt = waypoints(:, i);
        end_pt = waypoints(:, i+1);
        
        % Generate Linear Segment
        [t_seg, pts_seg] = get_task_space_trajectory(start_pt, end_pt, segment_duration, dt);
        
        % Add to main list
        if i == 1
            points_3d = pts_seg;
            time_steps = t_seg;
        else
            points_3d = [points_3d, pts_seg(:, 2:end)];
            time_steps = [time_steps, t_seg(2:end) + current_time];
        end
        
        current_time = time_steps(end);
        
        % --- ADD CORNER PAUSE (Visual Sharpness) ---
        % Repeats the corner point for 1 second so the robot stops briefly
        pause_duration = 1.0;
        pause_steps = floor(pause_duration / dt);
        
        pause_pts = repmat(end_pt, 1, pause_steps);
        pause_time = (1:pause_steps) * dt + current_time;
        
        points_3d = [points_3d, pause_pts];
        time_steps = [time_steps, pause_time];
        
        current_time = time_steps(end);
    end
    
    % (Optional) Close the loop D -> A if you want a full box
    % Uncomment below to close the square
    % [t_seg, pts_seg] = get_task_space_trajectory(pD, pA, segment_duration, dt);
    % points_3d = [points_3d, pts_seg(:, 2:end)];
    % time_steps = [time_steps, t_seg(2:end) + current_time];

else
    % Circle Mode Fallback
    circle_center = [0.15; 0.0; 0.15]; 
    circle_radius = 0.03;
    [time_steps, points_3d] = get_circular_trajectory(circle_center, circle_radius, 10.0, dt);
end

num_steps = length(time_steps);

% --- 4. SOLVE IK WITH ORIENTATION ---
fprintf('Solving IK for %d points...\n', num_steps);

q_data = zeros(4, num_steps);

% Initial Guess: Wrist Down
q_prev = [0; 0; 0; 1.57]; 
q_prev = inverse_kinematics_with_orientation(q_prev, points_3d(:,1));

for i = 1:num_steps
    target = points_3d(:, i);
    q_sol = inverse_kinematics_with_orientation(q_prev, target);
    q_data(:, i) = q_sol;
    q_prev = q_sol;
end

% --- 5. EXPORT FOR SIMULINK (.mat) ---
q1_ts = timeseries(q_data(1, :)', time_steps);
q2_ts = timeseries(q_data(2, :)', time_steps);
q3_ts = timeseries(q_data(3, :)', time_steps);
q4_ts = timeseries(q_data(4, :)', time_steps);


