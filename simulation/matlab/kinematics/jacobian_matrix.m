function J = jacobian_matrix(q)
% JACOBIAN_MATRIX - Computes numeric 6x4 Jacobian for 4-DOF manipulator
% Input: q = [q1; q2; q3; q4] (4x1 joint angles in radians)
% Output: J = 6x4 numeric Jacobian (linear + angular velocity)

% Robot link lengths
l1 = 0.04355;
l2 = 0.140;
l3 = 0.1324;
l4 = 0.01213;

% Build DH table
DH = [ q(1)        l1    0    pi/2;
       q(2)+pi/2    0    l2   pi;
       q(3)         0    l3   pi;
       q(4)         0    l4   0];

n = size(DH,1);

% Initialize
T = eye(4);
P = zeros(3,n+1);  % positions of joints
Z = zeros(3,n+1);  % z-axis of each joint
Z(:,1) = [0;0;1];  % base frame z-axis

% Forward kinematics to get positions and axes
for i = 1:n
    theta = DH(i,1);
    d     = DH(i,2);
    a     = DH(i,3);
    alpha = DH(i,4);
    
    % DH transformation
    T_i = transformation_func(theta,d,a,alpha);
    T = T*T_i;
    
    P(:,i+1) = T(1:3,4);   % position of current joint
    Z(:,i+1) = T(1:3,3);   % z-axis of current joint
end

% Compute 6x4 Jacobian
J = zeros(6,n);
for i = 1:n
    J(1:3,i) = cross(Z(:,i), P(:,end)-P(:,i));  % linear velocity
    J(4:6,i) = Z(:,i);                           % angular velocity
end
end