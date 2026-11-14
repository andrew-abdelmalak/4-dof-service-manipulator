%% test_forward_position.m
% Validates the forward position kinematics function 'fk_pos_func.m'.

clear;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'kinematics'));

% --- 1) Check if the function to test exists ---
if ~exist('fk_pos_func.m', 'file')
    error('Function ''fk_pos_func.m'' not found.');
end

% --- 2) Define Test Case ---
% At q = [0, 0, 0, 0], the arm points straight up:
% Z = l1 + l2 + l3 + l4 = 0.04355 + 0.140 + 0.133 + 0.109 = 0.42555 m.
q_test = [0.0, 0.0, 0.0, 0.0];      % 1x4 row vector
X_expected = [0.0; 0.0; 0.42555];   % 3x1 column vector

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

assert(error_magnitude < 1e-6, ...
    sprintf('FK position error %.4e exceeds 1e-6 m tolerance.', error_magnitude));
disp('SUCCESS: The fk_pos_func.m function matches expected position.');
