%% run_fk_4dof.m
% Generates symbolic forward kinematics functions based on the
% NEW HANDWRITTEN DH TABLE.



% --- 1) Define symbolic variables
nDOF = 4;
q = sym('q', [1 nDOF], 'real');
q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4);

% Define link lengths
l1 = 0.04355; % From photo (43.55)
l2 = 0.140;
l3 = 0.133;
l4 = 0.109;

% --- 2) Define the NEW Handwritten DH Table
% [ theta, d, a, alpha ]
DH = [ q1,         l1,  l2, pi/2;
       pi/2 + q2,  0,   l2, pi;
       q3,         0,   l3, pi;
       q4,         0,   l4, 0    ];

% --- 3) Compute symbolic forward kinematics
[T_total_sym, pos_sym, T_cells_sym] = forward_kinematics_func(DH);

disp('Symbolic end-effector position [X; Y; Z]:');
disp(pos_sym);

% --- 4) Generate Simulink-ready MATLAB function
% This creates 'fk_pos_func.m'
disp('Generating fk_pos_func.m...');
matlabFunction(pos_sym, 'File', 'fk_pos_func', 'Vars', { q });
disp('fk_pos_func.m has been created successfully.');

% --- 5) Save a test value for other scripts
q_vals = [0.1, 0.2, 0.3, 0.4];
save('fk_test_results_4dof.mat', 'q_vals', 'DH');
disp('Test values saved.');