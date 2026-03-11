%% test_inverse_kinematics.m
% Validates the inverse_kinematics_func.m function.



% --- 1) Ensure all required functions exist ---
% We need the FK function from MS02 and the Jacobian functions from MS03
if ~exist('fk_pos_func.m', 'file')
    error('MS02 function fk_pos_func.m not found. Please run run_fk_4dof.m first.');
end
if ~exist('jacobian_matrix.m', 'file')
    error('MS03 function jacobian_matrix.m not found.');
end
if ~exist('inverse_jacobian_matrix.m', 'file')
    error('MS03 function inverse_jacobian_matrix.m not found.');
end

% --- 2) Define Test Case ---
% We will test the "zero-joint configuration" from the project.
% At q = [0,0,0,0], the end-effector should be at [0, 0, 0.4255]
% [cite: 1, 1920]

% Initial guess for joint angles (1x4 row vector)
q0 = [0.0, 0.0, 0.0, 0.0];

% Desired end-effector position (3x1 column vector)
X_d = [0.0; 0.0; 0.4255]; 

fprintf('--- Inverse Kinematics Test ---\n');
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

% Call MS02 Forward Kinematics
X_actual = fk_pos_func(q_final);

fprintf('Actual Position (X_actual): [%.6f; %.6f; %.6f]\n', X_actual);

% --- 5) Calculate and Display Final Error ---
error_vec = X_d - X_actual;
error_magnitude = norm(error_vec);

fprintf('\n--- Verification ---\n');
fprintf('Final Position Error (X_d - X_actual):\n');
disp(error_vec);
fprintf('Error Magnitude (norm): %.4e meters\n', error_magnitude);

if error_magnitude < 1e-6 % Use the tolerance from your function
    disp('SUCCESS: The IK solver converged to the correct position.');
else
    disp('WARNING: Error is larger than tolerance. Check calculations.');
end