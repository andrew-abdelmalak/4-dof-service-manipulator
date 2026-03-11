function [time_vec, points] = get_square_path(start_pos, end_pos, safe_z, duration_per_leg, dt)
% GET_SQUARE_PATH Generates an obstacle-avoidance path (Up -> Across -> Down).
%
% Inputs:
%   start_pos: [x; y; z] Start coordinates
%   end_pos:   [x; y; z] Target coordinates
%   safe_z:    Height (z) to clear obstacles (e.g., 0.25)
%   duration_per_leg: Time (seconds) for each of the 3 segments
%   dt:        Time step (e.g., 0.1)
%
% Outputs:
%   time_vec:  Array of time steps
%   points:    3xN matrix of [x; y; z] coordinates

    % --- 1. Define Waypoints ---
    % P1: Start Position
    p1 = start_pos;
    
    % P2: Lift UP (Start X, Start Y, Safe Z)
    p2 = [start_pos(1); start_pos(2); safe_z];
    
    % P3: Move ACROSS (Target X, Target Y, Safe Z)
    p3 = [end_pos(1); end_pos(2); safe_z];
    
    % P4: Lower DOWN (Target Position)
    p4 = end_pos;
    
    % List of waypoints
    waypoints = [p1, p2, p3, p4];
    
    % --- 2. Generate Trajectory ---
    points = [];
    time_vec = [];
    current_time = 0;
    
    % Loop through the 3 segments (Lift, Across, Lower)
    for i = 1:(size(waypoints, 2) - 1)
        pt_a = waypoints(:, i);
        pt_b = waypoints(:, i+1);
        
        % Generate Linear Segment
        % (Logic: p(t) = p_start + (p_end - p_start) * (t/T))
        t_seg = 0:dt:duration_per_leg;
        num_pts = length(t_seg);
        pts_seg = zeros(3, num_pts);
        
        for k = 1:3
            pts_seg(k, :) = linspace(pt_a(k), pt_b(k), num_pts);
        end
        
        % Append to main list
        if i == 1
            points = pts_seg;
            time_vec = t_seg;
        else
            % Skip first point to avoid duplicate with previous segment
            points = [points, pts_seg(:, 2:end)];
            time_vec = [time_vec, t_seg(2:end) + current_time];
        end
        
        current_time = time_vec(end);
    end
end