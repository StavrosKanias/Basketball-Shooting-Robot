function [q,qd,qdd] = polynomial_trajectory(qi, qf, t, qdi, qdf)
    
    % COMPUTE JOINT SPACE TRAJECTORY USING CUBIC POLYNOMIAL

    % [q,qd,qdd] = polynomial_trajectory(qi, qf, t) computes the pose, velocity, and acceleration in the
    % joint space for a 7-DOF robotic arm using a cubic (3d order) polynomial. For the calculation of the 
    % polynomial coefficient the function uses the formulas described in page 175 of 'Robotics: Modelling,
    % Planning and Control (Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, Giuseppe Oriolo)'. The number 
    % of steps in the trajectory is defined by the length of the time vector T (Mx1). The trajectory Q, QD 
    % and QDD are MxN matrices, with one row per time step and one column per joint. Joint velocity and 
    % acceleration can be optionally returned as qd (MxN) and qdd (MxN) respectively.

    % [q,qd,qdd] = polynomial_trajectory(qi, qf, t, qdi, qdf) can be used to also define the initial and 
    % final joint velocity for the trajectory. By default these values are
    % set to 0.

    % If initial and desired velocities are not given set them to 0
    if nargin == 3
        qdi = zeros(1,7);
        qdf = zeros(1,7);
    end

    % Compute the polynomial coefficients
    a0 = qi;
    a1 = qdi;
    a2 = zeros(1,7);
    a3 = zeros(1,7);
    % Compute a2 and a3 for each joint by solving a 2x2 system
    tf = max(t);
    syms  a21 a31
    [sola21,sola31] = solve(a31*tf^3+a21*tf^2+a1(1)*tf+a0(1)==qf(1), 3*a31*tf^2+2*a21*tf+a1(1)==qdf(1));
    a2(1) = sola21;
    a3(1) = sola31;
    syms  a22 a32
    [sola22,sola32] = solve(a32*tf^3+a22*tf^2+a1(2)*tf+a0(2)==qf(2), 3*a32*tf^2+2*a22*tf+a1(2)==qdf(2));
    a2(2) = sola22;
    a3(2) = sola32;
    syms  a23 a33
    [sola23,sola33] = solve(a33*tf^3+a23*tf^2+a1(3)*tf+a0(3)==qf(3), 3*a33*tf^2+2*a23*tf+a1(3)==qdf(3));
    a2(3) = sola23;
    a3(3) = sola33;
    syms  a24 a34
    [sola24,sola34] = solve(a34*tf^3+a24*tf^2+a1(4)*tf+a0(4)==qf(4), 3*a34*tf^2+2*a24*tf+a1(4)==qdf(4));
    a2(4) = sola24;
    a3(4) = sola34;
    syms  a25 a35
    [sola25,sola35] = solve(a35*tf^3+a25*tf^2+a1(5)*tf+a0(5)==qf(5), 3*a35*tf^2+2*a25*tf+a1(5)==qdf(5));
    a2(5) = sola25;
    a3(5) = sola35;
    syms  a26 a36
    [sola26,sola36] = solve(a36*tf^3+a26*tf^2+a1(6)*tf+a0(6)==qf(6), 3*a36*tf^2+2*a26*tf+a1(6)==qdf(6));
    a2(6) = sola26;
    a3(6) = sola36;
    syms  a27 a37
    [sola27,sola37] = solve(a37*tf^3+a27*tf^2+a1(7)*tf+a0(7)==qf(7), 3*a37*tf^2+2*a27*tf+a1(7)==qdf(7));
    a2(7) = sola27;
    a3(7) = sola37;

    % Create the time part of the polynomial
    tt = [t'.^3 t'.^2 t' ones(size(t'))];
    % Create the coefficient part of the polynomial
    c = [a3' a2' a1' a0']';
    
    % Calculate q by multiplying the time part with the coefficient part of
    % the polynomial
    q = round(tt*c,3);

    % Compute optional velocity by deriving the polynomial once
    if nargout >= 2
        c = [ zeros(1,7)' 3*a3' 2*a2' a1' ]';
        qd = round(tt*c,3);
    end

    % Compute optional acceleration by deriving the polynomial twice
    if nargout == 3
        c = [ zeros(1,7)' zeros(1,7)' 6*a3' 2*a2']';
        qdd = round(tt*c,3);
    end