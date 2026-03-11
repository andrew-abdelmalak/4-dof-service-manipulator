function q_final = inverse_kinematics_func(q0, X_d)
%#codegen
% Implements the Newton-Raphson iterative inverse kinematics method.
% This version solves for 3D position ONLY.
%
% Inputs:
%   q0    : 4x1 or 1x4 initial joint position guess [q1, q2, q3, q4]
%   X_d   : 3x1 desired end-effector position vector [Xd; Yd; Zd]
%
% Output:
%   q_final : 4x1 final joint position vector [q1, q2, q3, q4]

max_iter = 100;         % Increase iterations
pos_tolerance = 1e-3;   % Relax tolerance (1mm error is fine)


q_k = q0(:).';     % 1x4 row vector
X_d = X_d(:);      % 3x1 column vector

for i = 1:max_iter
    % 1. Calculate current 3D position (using the handwritten DH model)
    X_k = forward_kinematics_func(q_k(1), q_k(2), q_k(3), q_k(4));

    % 2. Calculate 3D position error
    error_vec_pos = X_d - X_k; % 3x1 vector

    % 3. Check for convergence
    if norm(error_vec_pos) < pos_tolerance
        q_final = q_k(:); 
        return;
    end
    
    % 4. Get the FULL 6x4 spatial Jacobian
    J_spatial = jacobian_matrix(q_k);
    
    % 5. Extract ONLY the 3x4 linear Jacobian (Jv) - (Rows 1-3)
    J_linear = J_spatial(1:3, :);

    % 6. Get the 4x3 pseudo-inverse of the LINEAR Jacobian
    J_linear_pinv = pinv(J_linear); % This is 4x3

    % 7. Apply the Newton-Raphson update step (4x1) = (4x3) * (3x1)
    q_step = J_linear_pinv * error_vec_pos; 
    q_k = q_k + q_step.';

    % --- 8. CLAMP JOINTS TO +/- 90 DEGREES ---
    % Ensures q is never > pi/2 or < -pi/2
    q_max = 2.5;
    q_min = -2.5;
    q_k = min(q_k, q_max); % Clamp upper limit
    q_k = max(q_k, q_min);
end

fprintf('WARNING: IK did not converge after %g iterations. Final error = %.6e m\n', ...
        max_iter, norm(error_vec_pos));
q_final = q_k(:); 
end