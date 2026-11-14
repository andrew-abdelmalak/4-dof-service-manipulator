% full_tea_making_complete.m
% Sequence:
% 1. Start Center -> Get Spoon (Right) -> Return Center
% 2. Center -> Get Sugar (Left) -> Return Center
% 3. Stir
% 4. Stop and Stay Still
% 5. SAVE q1-q4 to .MAT

% --- 1. CONFIGURATION ---
dt = 0.1; 
segment_time = 2.0; % Duration for linear moves

% -- Coordinate Setup --
pos_cup         = [0.15;  0.00; 0.10]; % Center (Home)
pos_spoon_right = [0.15; -0.10; 0.10]; % Right
pos_sugar_left  = [0.15;  0.10; 0.10]; % Left
safe_z          = 0.22;                % Clearance height

% -- Stirring Setup --
stir_radius = 0.03;
stir_loops  = 4;
stir_time   = 4.0;

% -- Hold Setup --
hold_time   = 3.0; % How long to stay still at the end

% --- 2. TRAJECTORY GENERATION ---
points_3d = [];
time_steps = [];
current_time = 0;

fprintf('Generating Trajectory...\n');

% -- SEQUENCE 1: FETCH SPOON (Center -> Right -> Center) --
% A. Go to Spoon
[t1, p1] = get_square_path(pos_cup, pos_spoon_right, safe_z, segment_time, dt);
points_3d = [points_3d, p1];
time_steps = [time_steps, t1];
current_time = time_steps(end);

% B. Return to Cup
[t2, p2] = get_square_path(pos_spoon_right, pos_cup, safe_z, segment_time, dt);
points_3d = [points_3d, p2(:, 2:end)]; % Skip duplicate start point
time_steps = [time_steps, t2(2:end) + current_time];
current_time = time_steps(end);

% -- SEQUENCE 2: FETCH SUGAR (Center -> Left -> Center) --
% A. Go to Sugar
[t3, p3] = get_square_path(pos_cup, pos_sugar_left, safe_z, segment_time, dt);
points_3d = [points_3d, p3(:, 2:end)];
time_steps = [time_steps, t3(2:end) + current_time];
current_time = time_steps(end);

% B. Return to Cup
[t4, p4] = get_square_path(pos_sugar_left, pos_cup, safe_z, segment_time, dt);
points_3d = [points_3d, p4(:, 2:end)];
time_steps = [time_steps, t4(2:end) + current_time];
current_time = time_steps(end);

% -- SEQUENCE 3: STIR (Circle at Center) --
t_stir = 0:dt:stir_time;
theta = linspace(0, stir_loops * 2 * pi, length(t_stir));
x = pos_cup(1) + stir_radius * cos(theta);
y = pos_cup(2) + stir_radius * sin(theta);
z = pos_cup(3) * ones(size(x));

p_stir = [x; y; z];
points_3d = [points_3d, p_stir]; 
time_steps = [time_steps, t_stir + current_time];
current_time = time_steps(end);

% -- SEQUENCE 4: STOP AND STAY STILL --
% Repeat the last position for 'hold_time'
num_hold_steps = round(hold_time / dt);
last_pos = points_3d(:, end);
p_hold = repmat(last_pos, 1, num_hold_steps);
t_hold = (1:num_hold_steps) * dt;

points_3d = [points_3d, p_hold];
time_steps = [time_steps, t_hold + current_time];


% --- 3. INVERSE KINEMATICS ---
num_steps = length(time_steps);
fprintf('Solving IK for %d points...\n', num_steps);

q_data = zeros(4, num_steps);
q_prev = [0; 0; 0; 0]; 

for i = 1:num_steps
    target_point = points_3d(:, i);
    % Make sure 'inverse_kinematics_with_orientation' is in your path
    q_sol = inverse_kinematics_with_orientation(q_prev, target_point); 
    q_data(:, i) = q_sol;
    q_prev = q_sol;
end

% --- 4. SAVE DATA (q1-q4) ---
fprintf('Saving Data to .MAT...\n');

% Separate the rows into individual variables
q1 = q_data(1, :);
q2 = q_data(2, :);
q3 = q_data(3, :);
q4 = q_data(4, :);

% Create TimeSeries objects (For Simulink)
q1_ts = timeseries(q1', time_steps);
q2_ts = timeseries(q2', time_steps);
q3_ts = timeseries(q3', time_steps);
q4_ts = timeseries(q4', time_steps);

% Save to .MAT file
filename_mat = 'robot_tea_data.mat';
save(filename_mat, 'q1_ts', 'q2_ts', 'q3_ts', 'q4_ts', 'q1', 'q2', 'q3', 'q4', 'time_steps');

fprintf('DONE. Data saved to: %s\n', filename_mat);