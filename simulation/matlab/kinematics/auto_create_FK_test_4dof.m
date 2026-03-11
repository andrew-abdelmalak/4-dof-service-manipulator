% auto_create_FK_test_4dof.m
modelName = 'FK_test_4dof';
new_system(modelName); open_system(modelName);

% Add Constant block
add_block('simulink/Sources/Constant', [modelName '/q_const']);
set_param([modelName '/q_const'], 'Position', [30 80 100 120]);
set_param([modelName '/q_const'], 'Value', 'q_vals');
set_param([modelName '/q_const'], 'OutDataTypeStr', 'double');
set_param([modelName '/q_const'], 'PortDimensions', '4');

% Add MATLAB Function block
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/FK_pos']);
set_param([modelName '/FK_pos'], 'Position', [170 70 320 150]);
matlabFunctionText = sprintf([...
"function pos = fcn(q)\n" ...
"%% Calls generated fk_pos_func (expects [q1 q2 q3 q4])\n" ...
"pos = fk_pos_func(q(:).');\n" ...
"end"]);
set_param([modelName '/FK_pos'], 'Script', matlabFunctionText);

% Add To Workspace
add_block('simulink/Sinks/To Workspace', [modelName '/pos_fk']);
set_param([modelName '/pos_fk'], 'Position', [420 80 480 120]);
set_param([modelName '/pos_fk'], 'VariableName', 'pos_fk');
set_param([modelName '/pos_fk'], 'SaveFormat', 'Array');

% Connect lines
add_line(modelName, 'q_const/1', 'FK_pos/1');
add_line(modelName, 'FK_pos/1', 'pos_fk/1');

% Save and open
save_system(modelName);
open_system(modelName);
disp(['Model ' modelName ' created. Set Stop time and Run.']);
