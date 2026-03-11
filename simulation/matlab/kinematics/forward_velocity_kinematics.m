function V_spatial = forward_velocity_kinematics(q, q_dot)
% FORWARD_VELOCITY_KINEMATICS - Computes end-effector velocity
% Input:
%   q     = [q1; q2; q3; q4]  (4x1 joint angles in radians)
%   q_dot = [dq1; dq2; dq3; dq4] (4x1 joint velocities)
% Output:
%   V_F = [vx; vy; vz; wx; wy; wz] 6x1 end-effector velocity

% Compute numeric Jacobian
J = jacobian_matrix(q);   % 6x4

% Compute end-effector velocity
V_spatial = J * q_dot;          % 6x1 vector
end