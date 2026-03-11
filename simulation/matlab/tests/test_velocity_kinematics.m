%% test_velocity_kinematics.m
% This script performs a full round-trip test on all kinematics functions,
% similar to the Python script 'ms3_validate_roundtrip.py'.
%
% Round-trip:
%   (q, q_dot) -> (x, V) -> (q_rec, q_dot_rec)
% and compare input vs output.


% --- 1) Ensure fk_pos_func.m exists ---
% We need the FK position function from MS02
if ~exist('fk_pos_func.m', 'file')
    try
        disp('Running run_fk_4dof.m to generate fk_pos_func.m...');
        run('run_fk_4dof.m');
        disp('fk_pos_func.m generated.');
    catch ME
        disp('ERROR: Could not find or run run_fk_4dof.m.');
        disp('Please ensure run_fk_4dof.m is on the MATLAB path.');
        rethrow(ME);
    end
end

% --- 2) Define Test Cases ---
% A few sample joint configs (in radians) and velocities (rad/s)
test_cases = {
    % q,                       q_dot
    {[0.0, 0.0, 0.0, 0.0],     [0.1, 0.0, 0.0, 0.0]};
    {[0.2, 0.4, -0.3, 0.1],    [0.0, 0.2, -0.1, 0.05]};
    {[-0.3, 0.6, 0.2, -0.2],   [0.1, -0.15, 0.05, 0.0]};
};

% --- 3) Run Tests ---
for i = 1:length(test_cases)
    q_in = test_cases{i}{1};
    q_dot_in = test_cases{i}{2};
    
    fprintf('=== New test case %d ===\n', i);
    print_vec('q (input)', q_in);
    print_vec('q_dot (input)', q_dot_in);
    
    % 1) Forward position (MS2)
    x = fk_pos_func(q_in); % 3x1
    print_vec('x = FK(q)', x);
    
    % 2) Forward velocity (MS3)
    V = forward_velocity_kinematics(q_in, q_dot_in); % 6x1
    print_vec('V = J(q) * q_dot', V);
    
    % 3) Inverse position: recover q from x
    %    Use the true q as initial guess so it converges back to it.
    q_rec = inverse_kinematics_func(q_in, x); % 1x4
    print_vec('q_rec from IK(x)', q_rec);
    
    % 4) Inverse velocity: recover q_dot from (q_rec, V)
    q_dot_rec = inverse_velocity_kinematics(q_rec, V); % 4x1
    print_vec('q_dot_rec from J_pinv * V', q_dot_rec);
    
    % 5) Errors
    dq = q_rec(:).' - q_in(:).';
    dq_dot = q_dot_rec(:).' - q_dot_in(:).';
    
    print_vec('q_rec - q', dq);
    print_vec('q_dot_rec - q_dot', dq_dot);
    fprintf('||q_rec - q|| = %.6e\n', norm(dq));
    fprintf('||q_dot_rec - q_dot|| = %.6e\n', norm(dq_dot));
    disp(' ');
end

% --- Helper function for printing ---
function print_vec(name, v)
    v = v(:).'; % Ensure v is a row vector
    str = sprintf(' %.6f,', v);
    str = str(1:end-1); % Remove trailing comma
    fprintf('%s: [%s]\n', pad(name, 22), str);
end