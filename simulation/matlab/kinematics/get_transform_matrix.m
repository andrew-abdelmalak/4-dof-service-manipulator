function T_total = get_transform_matrix(q1, q2, q3, q4)
% Calculates the full 4x4 Homogeneous Transformation Matrix
% Used to extract the current Z-axis orientation vector.

    % Link Lengths (Must match your existing files)
    d1 = 0.04355; 
    a2 = 0.14000; 
    a3 = 0.13240; 
    a4 = 0.01213; 

    % DH Transformations (Matches your forward_kinematics_func)
    T0_1 = transformation_func(q1,      d1, 0.0,  pi/2);
    T1_2 = transformation_func(pi/2+q2, 0.0,  a2, pi);
    T2_3 = transformation_func(q3,      0.0,  a3, pi);
    T3_4 = transformation_func(q4,      0.0,  a4, 0.0);

    % Total Transformation
    T_total = T0_1 * T1_2 * T2_3 * T3_4;
end