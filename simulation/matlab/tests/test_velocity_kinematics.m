%% test_velocity_kinematics.m
% Performs a full round-trip test on all kinematics functions:
%   (q, q_dot) -> (x, V) -> (q_rec, q_dot_rec)
% and compares input vs output.

clear;
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'kinematics'));

if ~exist('fk_pos_func.m', 'file')
    error('Function fk_pos_func.m not found.');
end

print_vec = @(name, v) fprintf('%-22s: [%s]\n', name, strjoin(arrayfun(@(x) sprintf('%.6f', x), v(:).', 'UniformOutput', false), ', '));

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
    
    % 1) Forward position (Position)
    x = fk_pos_func(q_in); % 3x1
    print_vec('x = FK(q)', x);
    
    % 2) Forward velocity (Velocity)
    V = forward_velocity_kinematics(q_in, q_dot_in); % 6x1
    print_vec('V = J(q) * q_dot', V);
    
    % 3) Inverse position: recover q from x
    %    Use the true q as initial guess so it converges back to it.
    q_rec = inverse_kinematics_func(q_in, x); % 1x4
    print_vec('q_rec from IK(x)', q_rec);
    
    % 4) Inverse velocity: recover q_dot from (q_rec, V)
    q_dot_rec = inverse_velocity_kinematics(q_rec, V); % 4x1
    print_vec('q_dot_rec from J_pinv * V', q_dot_rec);
    
    % 5) Errors & Assertions
    dq = q_rec(:).' - q_in(:).';
    dq_dot = q_dot_rec(:).' - q_dot_in(:).';
    V_rec = jacobian_matrix(q_rec) * q_dot_rec(:);
    dV = norm(V_rec - V);

    print_vec('q_rec - q', dq);
    print_vec('q_dot_rec - q_dot', dq_dot);
    fprintf('||q_rec - q|| = %.6e\n', norm(dq));
    fprintf('||q_dot_rec - q_dot|| = %.6e\n', norm(dq_dot));
    fprintf('||J*q_dot_rec - V|| = %.6e\n\n', dV);

    assert(dV < 1e-12, sprintf('Case %d twist residual %.3e >= 1e-12', i, dV));
    if i >= 2
        assert(norm(dq_dot) < 1e-12, sprintf('Case %d velocity roundtrip error %.3e >= 1e-12', i, norm(dq_dot)));
    end
end
disp('SUCCESS: All velocity kinematics roundtrip checks passed.');
