    function [time_steps, points] = get_task_space_trajectory(start_pt, end_pt, total_time, dt)
% GET_TASK_SPACE_TRAJECTORY Generates a Linear Task-Space Trajectory.
% Formula: X(t) = X0 + velocity * t
%
% Inputs:
%   start_pt   : [x; y; z] Start position column vector
%   end_pt     : [x; y; z] End position column vector
%   total_time : Duration of the movement (seconds)
%   dt         : Time step (seconds)
%
% Outputs:
%   time_steps : 1xN vector of time values
%   points     : 3xN matrix of trajectory points [x; y; z]

    % Calculate number of steps
    steps = floor(total_time / dt);
    
    % Ensure inputs are column vectors (3x1) to prevent dimension errors
    start_pt = start_pt(:);
    end_pt = end_pt(:);

    % Calculate constant velocity vector for this segment
    velocity_vector = (end_pt - start_pt) / total_time;

    % Generate time vector
    time_steps = 0:dt:(steps * dt);
    num_points = length(time_steps);
    
    % Pre-allocate points matrix for speed
    points = zeros(3, num_points);

    % Calculate points
    for i = 1:num_points
        t = time_steps(i);
        % Linear equation: P = P0 + v*t
        current_pos = start_pt + velocity_vector * t;
        points(:, i) = current_pos;
    end
end