%% test_inverse_kinematics.m
% Validates the inverse_kinematics_func.m function.

clear;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'kinematics'));

% --- 1) Ensure all required functions exist ---
if ~exist('fk_pos_func.m', 'file')
    error('Function fk_pos_func.m not found.');
end
if ~exist('jacobian_matrix.m', 'file')
    error('Function jacobian_matrix.m not found.');
end
if ~exist('inverse_jacobian_matrix.m', 'file')
    error('Function inverse_jacobian_matrix.m not found.');
end

% --- 2) Test Case 1: Zero-joint configuration ---
% At q = [0, 0, 0, 0], the end-effector is at [0; 0; 0.42555] m.
q0 = [0.0, 0.0, 0.0, 0.0];
X_d = [0.0; 0.0; 0.42555];

fprintf('--- Inverse Kinematics Test (Case 1: Zero Config) ---\n');
fprintf('Initial Guess (q0): [%.4f, %.4f, %.4f, %.4f]\n', q0);
fprintf('Desired Position (X_d): [%.4f; %.4f; %.4f]\n', X_d);

% --- 3) Run the Inverse Kinematics Solver ---

    % Call the function under test
    q_final = inverse_kinematics_func(q0, X_d);
    q_final_row = q_final(:).';
    
    fprintf('\n--- Results ---\n');
    fprintf('Solved Angles (q_final): [%.6f, %.6f, %.6f, %.6f]\n', q_final_row);

% catch ME
%     disp('ERROR: inverse_kinematics_func failed.');
%     rethrow(ME);
% end

% --- 4) Verification Step ---
% To check if q_final is correct, we run it back through
% forward kinematics and see if we get our desired position X_d.

% Call Forward Kinematics
X_actual = fk_pos_func(q_final(:).');

fprintf('Actual Position (X_actual): [%.6f; %.6f; %.6f]\n', X_actual);

% --- 5) Calculate and Display Final Error ---
error_vec = X_d - X_actual;
error_magnitude = norm(error_vec);

fprintf('\n--- Verification (Case 1) ---\n');
fprintf('Final Position Error (norm): %.4e meters\n', error_magnitude);

assert(error_magnitude < 1e-3, ...
    sprintf('IK Case 1 error %.4e exceeds 1e-3 m tolerance.', error_magnitude));

% --- 6) Test Case 2: Cold-start IK away from singularity ---
X_d2 = [-0.21403838; -0.04338809; 0.35016419];
fprintf('\n--- Inverse Kinematics Test (Case 2: Cold Start) ---\n');
q_final2 = inverse_kinematics_func(q0, X_d2);
X_actual2 = fk_pos_func(q_final2(:).');
error_magnitude2 = norm(X_d2 - X_actual2);
fprintf('Final Position Error (norm): %.4e meters\n', error_magnitude2);

assert(error_magnitude2 < 1e-3, ...
    sprintf('IK Case 2 error %.4e exceeds 1e-3 m tolerance.', error_magnitude2));
disp('SUCCESS: All IK tests converged within tolerance.');