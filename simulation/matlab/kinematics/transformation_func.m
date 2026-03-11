function T = transformation_func(theta, d, a, alpha)

    % If any input is symbolic, ensure all arithmetic stays symbolic
    if isa(theta, 'sym') || isa(d, 'sym') || isa(a, 'sym') || isa(alpha, 'sym')
        % Convert numeric inputs to symbolic so operations remain symbolic
        if ~isa(theta, 'sym'), theta = sym(theta); end
        if ~isa(d, 'sym'),     d     = sym(d);     end
        if ~isa(a, 'sym'),     a     = sym(a);     end
        if ~isa(alpha, 'sym'), alpha = sym(alpha); end
    end

    % Precompute trig terms (keeps expressions compact and readable)
    cth = cos(theta);
    sth = sin(theta);
    cal = cos(alpha);
    sal = sin(alpha);

    % Build the homogeneous transform matrix using the standard DH formula
    T = [...
        cth,    -sth*cal,   sth*sal,    a*cth;
        sth,     cth*cal,  -cth*sal,    a*sth;
        0,       sal,        cal,        d;
        0,       0,          0,          1];

    % Simplify the output a bit if symbolic (helps later algebra)
    if isa(T, 'sym')
        T = simplify(T);
    end
end
