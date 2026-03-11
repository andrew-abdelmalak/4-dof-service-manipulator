%% create_jacobian_func.m
% This script derives the symbolic linear Jacobian and creates the 
% function 'jacobian_matrix.m' as required by Milestone 03, Task 1.

% --- 1) Run FK script to get symbolic variables
% This script (run_fk_4dof.m) must be in the same folder.
% It will create 'q' (1x4 sym vector) and 'pos_sym' (3x1 sym vector)
% It also creates 'q_vals' (1x4 numeric vector) for testing.
try
    disp('Running run_fk_4dof.m to load symbolic variables...');
    % We use 'run' to execute the script in the current workspace
    run('run_fk_4dof.m');
    disp('Symbolic variables (q, pos_sym, q_vals) loaded.');
catch ME
    disp('ERROR: Could not run run_fk_4dof.m.');
    disp('Please ensure run_fk_4dof.m, forward_kinematics_func.m, and transformation_func.m are all in the same directory.');
    rethrow(ME);
end

% --- 2) Calculate Symbolic Linear Jacobian
% We calculate J_linear = d(pos_sym) / d(q)
% This will be a 3x4 matrix for a 3D position vector and 4 joints.
disp('Calculating symbolic linear Jacobian (J_linear = d(pos)/d(q))...');

% 'jacobian(f, v)' computes the Jacobian of f with respect to v.
% f = pos_sym (our 3x1 position vector)
% v = q (our 1x4 joint vector)
% The result J_sym_linear will be a 3x4 matrix.
J_sym_linear = jacobian(pos_sym, q);

disp('Symbolic linear Jacobian (J_sym_linear):');


%% create_jacobian_func.m
% This script derives the symbolic linear Jacobian and creates the 
% function 'jacobian_matrix.m' as required by Milestone 03, Task 1.
% --- 1) Run FK script to get symbolic variables
% This script (run_fk_4dof.m) must be in the same folder.
% It will create 'q' (1x4 sym vector) and 'pos_sym' (3x1 sym vector)
% It also creates 'q_vals' (1x4 numeric vector) for testing.
try
    disp('Running run_fk_4dof.m to load symbolic variables...');
    % We use 'run' to execute the script in the current workspace
    run('run_fk_4dof.m');
    disp('Symbolic variables (q, pos_sym, q_vals) loaded.');
catch ME
    disp('ERROR: Could not run run_fk_4dof.m.');
    disp('Please ensure run_fk_4dof.m, forward_kinematics_func.m, and transformation_func.m are all in the same directory.');
    rethrow(ME);
end
% --- 2) Calculate Symbolic Linear Jacobian
% We calculate J_linear = d(pos_sym) / d(q)
% This will be a 3x4 matrix for a 3D position vector and 4 joints.
disp('Calculating symbolic linear Jacobian (J_linear = d(pos)/d(q))...');
% 'jacobian(f, v)' computes the Jacobian of f with respect to v.
% f = pos_sym (our 3x1 position vector)
% v = q (our 1x4 joint vector)
% The result J_sym_linear will be a 3x4 matrix.
J_sym_linear = jacobian(pos_sym, q);

% --- EDITED SECTION ---
% Display the symbolic matrix in a human-readable format
disp('--- Symbolic Linear Jacobian (pretty format) ---');
pretty(J_sym_linear);
disp('--------------------------------------------------');
% --- END EDIT ---

% --- 3) Generate MATLAB Function File
% This creates the file 'jacobian_matrix.m'
disp('Generating jacobian_matrix.m function file...');
matlabFunction(J_sym_linear, 'File', 'jacobian_matrix', 'Vars', {q});
disp('File jacobian_matrix.m created successfully.');
% --- 4) Quick Numeric Test
% We can test the newly generated function using the 'q_vals'
% that were loaded from run_fk_4dof.m
disp('Performing numeric test of jacobian_matrix(q_vals)...');
try
    % Ensure q_vals is a row vector [q1 q2 q3 q4] as expected by the generated func
    q_vals_row = q_vals(:).'; 
    J_num = jacobian_matrix(q_vals_row);
    disp('Numeric Jacobian result from jacobian_matrix(q_vals):');
    disp(J_num);
catch ME
    disp('ERROR: Could not run the newly generated jacobian_matrix.m function.');
    disp('This might happen if the file was not created correctly.');
    rethrow(ME);
end
disp('Task 1.1 (Jacobian) complete. You now have jacobian_matrix.m');



% --- 3) Generate MATLAB Function File
% This creates the file 'jacobian_matrix.m'
disp('Generating jacobian_matrix.m function file...');
matlabFunction(J_sym_linear, 'File', 'jacobian_matrix', 'Vars', {q});
disp('File jacobian_matrix.m created successfully.');

% --- 4) Quick Numeric Test
% We can test the newly generated function using the 'q_vals'
% that were loaded from run_fk_4dof.m
disp('Performing numeric test of jacobian_matrix(q_vals)...');
try
    % Ensure q_vals is a row vector [q1 q2 q3 q4] as expected by the generated func
    q_vals_row = q_vals(:).'; 
    J_num = jacobian_matrix(q_vals_row);
    disp('Numeric Jacobian result from jacobian_matrix(q_vals):');
    disp(J_num);
catch ME
    disp('ERROR: Could not run the newly generated jacobian_matrix.m function.');
    disp('This might happen if the file was not created correctly.');
    rethrow(ME);
end

disp('Task 1.1 (Jacobian) complete. You now have jacobian_matrix.m');