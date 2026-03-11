function [time_steps, points] = get_circular_trajectory(center, radius, total_time, dt)
% Generates circular path points
    steps = floor(total_time / dt);
    center = center(:);
    z_height = center(3);
    
    time_steps = 0:dt:(steps*dt);
    num_points = length(time_steps);
    points = zeros(3, num_points);
    
    for i = 1:num_points
        t = time_steps(i);
        theta = (2 * pi / total_time) * t;
        x = center(1) + radius * cos(theta);
        y = center(2) + radius * sin(theta);
        points(:,i) = [x; y; z_height];
    end
end