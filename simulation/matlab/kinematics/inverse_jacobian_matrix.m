function J_pinv = inverse_jacobian_matrix(q)
%#codegen
% Calculates the 4x6 pseudo-inverse of the 6x4 spatial Jacobian.
% J_pinv = pinv(J(q))
%
% Input:
%   q : 4x1 or 1x4 joint position vector [q1, q2, q3, q4]
%
% Output:
%   J_pinv : 4x6 pseudo-inverse matrix

% 1. Get the 6x4 numeric spatial Jacobian
J = jacobian_matrix(q(:).'); % This returns the 6x4 matrix

% 2. Calculate and return the pseudo-inverse (will be 4x6)
J_pinv = pinv(J);
end