%% test_fk_run.m
% Quick test to call fk_pos_func and show output

% Add kinematics functions to path (adjust if running from a different location)
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'kinematics'));

% (re)generate fk_pos_func and q_vals if needed:
run('run_fk_4dof.m');   % this creates q_vals and fk_pos_func

% List workspace variables
who

% Call the generated FK function and display result
pos_from_fk = fk_pos_func(q_vals);
disp('pos_from_fk =');
disp(pos_from_fk);
