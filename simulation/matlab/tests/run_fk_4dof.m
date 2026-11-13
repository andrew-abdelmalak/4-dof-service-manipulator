%% run_fk_4dof.m
% Verifies (and optionally symbolically re-derives in MATLAB) the 4-DOF
% forward kinematics against the DH table and fk_pos_func.m.

clear;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'kinematics'));

has_sym = false;
try
    sym('x');
    has_sym = true;
catch
    has_sym = false;
end

if has_sym
    nDOF = 4;
    q = sym('q', [1 nDOF], 'real');
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);

    l1 = 0.04355;
    l2 = 0.140;
    l3 = 0.133;
    l4 = 0.109;

    % [ theta, d, a, alpha ]
    DH = [ q1,         l1,  0,  pi/2;
           pi/2 + q2,  0,   l2, pi;
           q3,         0,   l3, pi;
           q4,         0,   l4, 0    ];

    pos_sym = forward_kinematics_func(q1, q2, q3, q4);
    disp('Symbolic end-effector position [X; Y; Z]:');
    disp(pos_sym);
else
    fprintf('Symbolic Math Toolbox unavailable; running numeric FK consistency check.\n');
end

test_configs = [
    0.0,  0.0,  0.0,  0.0;
    0.1,  0.2,  0.3,  0.4;
   -0.3,  0.6,  0.2, -0.2;
    0.5, -0.4,  0.7,  0.1
];

max_diff = 0.0;
for k = 1:size(test_configs, 1)
    qk = test_configs(k, :);
    pos_dh = forward_kinematics_func(qk(1), qk(2), qk(3), qk(4));
    pos_gen = fk_pos_func(qk);
    diff_k = norm(pos_dh(:) - pos_gen(:));
    max_diff = max(max_diff, diff_k);
end

fprintf('Max |DH_FK - fk_pos_func| across %d configs: %.4e m\n', size(test_configs, 1), max_diff);
assert(max_diff < 1e-12, sprintf('FK mismatch %.4e exceeds 1e-12 m.', max_diff));
disp('SUCCESS: forward_kinematics_func and fk_pos_func agree.');