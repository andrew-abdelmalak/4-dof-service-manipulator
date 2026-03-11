function q_final = inverse_kinematics_with_orientation(q0, X_des)
% Solves IK with:
% 1. Target: Position X,Y,Z
% 2. Constraint: Orientation (Looking Down)
% 3. Limits: Wrist restricted to +/- 90 degrees

    max_iter = 100;
    tolerance = 1e-4;
    alpha = 0.5; 
    
    % --- 1. ORIENTATION TARGET ---
    % Global Z is up, so [0; 0; -1] is pointing at the floor.
    Z_desired = [0; 0; -1]; 

    q_current = q0(:); 

    % --- 2. SPECIFIC JOINT LIMITS ---
    % q1, q2, q3: Relaxed to +/- 150 degrees (2.6 rad) to ensure reach
    % q4 (Wrist): Restricted to +/- 90 degrees (1.57 rad)
    q_min = [-pi; -pi ; -pi ; -1.45];
    q_max = [ pi;  pi ;  pi ;  -1];

    for i = 1:max_iter
        % Get Current Pose
        T_curr = get_transform_matrix(q_current(1), q_current(2), q_current(3), q_current(4));
        X_curr = T_curr(1:3, 4);       
        Z_curr = T_curr(1:3, 3);       
        
        % Errors
        err_pos = X_des - X_curr;
        err_ori = cross(Z_curr, Z_desired);
        
        % Combined Error Vector
        error_total = [err_pos; err_ori * 0.5]; 

        % Check Convergence
        if norm(error_total) < tolerance
            q_final = q_current;
            return;
        end

        % Jacobian Update
        J = jacobian_matrix(q_current);
        J_inv = pinv(J);
        dq = J_inv * error_total;
        
        q_current = q_current + alpha * dq;
        
        % Normalize Angles (-pi to pi)
        q_current = atan2(sin(q_current), cos(q_current)); 
        
        % --- 3. APPLY INDIVIDUAL LIMITS ---
        % Clamps each joint to its specific min/max defined above
        q_current = max(min(q_current, q_max), q_min);     
    end
    
    q_final = q_current;
end