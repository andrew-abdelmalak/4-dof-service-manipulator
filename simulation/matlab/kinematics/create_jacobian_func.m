%% create_jacobian_func.m
% Derives (when Symbolic Math Toolbox is present) or numerically verifies
% the 3x4 linear Jacobian against the 6x4 analytical jacobian_matrix.m
% without overwriting jacobian_matrix.m.

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

q_vals = [0.1, 0.2, 0.3, 0.4];
eps_fd = 1e-7;

has_sym = false;
try
    sym('x');
    has_sym = true;
catch
    has_sym = false;
end

if has_sym
    q = sym('q', [1 4], 'real');
    pos_sym = forward_kinematics_func(q(1), q(2), q(3), q(4));
    J_sym_linear = jacobian(pos_sym, q);
    disp('--- Symbolic Linear Jacobian (3x4) ---');
    pretty(J_sym_linear);
    J_ref = double(subs(J_sym_linear, q, q_vals));
else
    fprintf('Symbolic Math Toolbox unavailable; verifying linear Jacobian via central FD.\n');
    J_ref = zeros(3, 4);
    for j = 1:4
        dq = zeros(1, 4);
        dq(j) = eps_fd;
        f_plus = fk_pos_func(q_vals + dq);
        f_minus = fk_pos_func(q_vals - dq);
        J_ref(:, j) = (f_plus(:) - f_minus(:)) / (2 * eps_fd);
    end
end

J_full = jacobian_matrix(q_vals);
J_lin_analytical = J_full(1:3, :);
max_err = max(abs(J_lin_analytical(:) - J_ref(:)));
fprintf('Max |J_analytical(1:3,:) - J_ref| error: %.4e\n', max_err);
assert(max_err < 1e-7, sprintf('Linear Jacobian mismatch %.4e >= 1e-7', max_err));
disp('SUCCESS: Linear Jacobian matches analytical jacobian_matrix(1:3,:).');
