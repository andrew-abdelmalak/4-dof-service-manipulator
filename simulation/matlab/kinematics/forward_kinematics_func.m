function pos = forward_kinematics_func(q1, q2, q3, q4)
%#codegen
% FORWARD_KINEMATICS_FUNC (Coder-Friendly Version)
% Calculates the Cartesian position [x; y; z] from 4 scalar joint angles.
% This implements the *exact* DH chain from your Python/Set 2 files.
% All inputs must be scalar angles in radians.

% Define Link Lengths (must be constants for codegen)
d1 = 0.04355; % base offset
a2 = 0.14000; % link 2
a3 = 0.13240; % link 3
a4 = 0.01213; % wrist

% Call transformation_func for each link
% This requires transformation_func.m to be on the MATLAB path.
% T0_1: [q1, d1, 0, pi/2]
T0_1 = transformation_func(q1,      d1, 0.0,  pi/2);

% T1_2: [pi/2+q2, 0, a2, pi]
T1_2 = transformation_func(pi/2+q2, 0.0,  a2, pi);

% T2_3: [q3, 0, a3, pi]
T2_3 = transformation_func(q3,      0.0,  a3, pi);

% T3_4: [q4, 0, a4, 0]
T3_4 = transformation_func(q4,      0.0,  a4, 0.0);

% Calculate total transformation
T_total = T0_1 * T1_2 * T2_3 * T3_4;

% Extract 3x1 position vector
pos = T_total(1:3, 4);

end

% --- Nested Helper Function ---
% We nest this function to make the file self-contained
% and guarantee Coder can find it.
function T = transformation_func(theta, d, a, alpha)
    cth = cos(theta);
    sth = sin(theta);
    cal = cos(alpha);
    sal = sin(alpha);

    T = [...
        cth,    -sth*cal,   sth*sal,    a*cth;
        sth,     cth*cal,  -cth*sal,    a*sth;
        0,       sal,        cal,        d;
        0,       0,          0,          1];
end