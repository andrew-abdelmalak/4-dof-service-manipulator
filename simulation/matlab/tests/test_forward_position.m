%% test_forward_position.m
% Validates the forward position kinematics function 'fk_pos_func.m'.
%
% This test assumes that 'run_fk_4dof.m' has already been run at least once
% to generate 'fk_pos_func.m' using the CORRECT DH parameters.

clear; clc; close all;

% --- 1) Check if the function to test exists ---
if ~exist('fk_pos_func.m', 'file')
    error(['Function ''fk_pos_func.m'' not found. ' ...
           'Please run your updated ''run_fk_4dof.m'' script first to generate it.']);
end

% --- 2) Define Test Case ---
% We will test the "zero-joint configuration".
% From Deliverables.pdf and milestone3_node.py, we know:
% q = [0, 0, 0, 0] should result in X = [0, 0, 0.4255]

q_test = [0.0, 0.0, 0.0, 0.0];      % 1x4 row vector
X_expected = [0.0; 0.0; 0.4255];    % 3x1 column vector

fprintf('--- Forward Position Kinematics Test ---\n');
fprintf('Test Angles (q_test): [%.4f, %.4f, %.4f, %.4f]\n', q_test);
fprintf('Expected Position (X_expected): [%.4f; %.4f; %.4f]\n', X_expected);

% --- 3) Run the Forward Kinematics Function ---
try
    % Call the function under test
    X_actual = fk_pos_func(q_test);
    
    fprintf('\n--- Results ---\n');
    fprintf('Actual Position (X_actual):   [%.6f; %.6f; %.6f]\n', X_actual);

catch ME
    disp('ERROR: fk_pos_func(q) failed to execute.');
    rethrow(ME);
end

% --- 4) Calculate and Display Final Error ---
error_vec = X_expected - X_actual;
error_magnitude = norm(error_vec);

fprintf('\n--- Verification ---\n');
fprintf('Final Position Error (X_expected - X_actual):\n');
disp(error_vec);
fprintf('Error Magnitude (norm): %.4e meters\n', error_magnitude);

if error_magnitude < 1e-2 % Use a small tolerance
    disp('SUCCESS: The fk_pos_func.m function is correct.');
else
    disp('WARNING: Error is large. Did you update and run run_fk_4dof.m with the correct DH table?');
end