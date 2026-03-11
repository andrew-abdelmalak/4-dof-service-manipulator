function q_dot = inverse_velocity_kinematics(q, V_des)
% Calculates inverse velocity kinematics using the pseudo-inverse of the Jacobian.
% q_dot = pinv(J(q)) * V_des
%
% Inputs:
%   q       : 4x1 or 1x4 joint position vector [q1, q2, q3, q4]
%   V_des   : 6x1 desired spatial velocity vector [Vx; Vy; Vz; Wx; Wy; Wz]
%
% Output:
%   q_dot : 4x1 joint velocity vector [q1d; q2d; q3d; q4d]

% 1. Get the 4x6 pseudo-inverse of the spatial Jacobian
%    (This calls inverse_jacobian_matrix.m, which calls jacobian_matrix.m)
J_pinv = inverse_jacobian_matrix(q(:).'); % force row for helper

% 2. Ensure V_des is a column vector (6x1)
V_des_col = V_des(:);

% 4. Calculate the 4x1 joint velocity vector
q_dot = J_pinv * V_des_col; % (4x6) * (6x1) -> (4x1)

end