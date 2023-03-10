clear; close all; clc;

% Initialize the Arduino microcontroller
s = initialize_arduino();

pause(1);

% Move to initial position
target = [0.7 -0.4 0]';
center  = [0.7 0 0]';
target_angle = rad2deg(atan2(cross(center,target), dot(center,target)));
z_target_angle = target_angle(3);

dt = 0.000001;
dq = [10 + z_target_angle, -12, 5, -7, 2, 18, 5];
dqi = [10 - z_target_angle, -12, 5, -7, 2, 18, 5];

move(s, deg2rad([dq 0]), dt);
pause(1);
move(s, deg2rad([dqi 0]), dt);

pause(2);

% Grab the ball
move(s(8), deg2rad(25), dt);

pause(1);

% Shoot
t = 0:dt:2;
qi = deg2rad([0 15 0 -30 0 -40 0] + dq);
qf = deg2rad([0 -20 0 0 0 -8 0] + dq);
qdi = deg2rad([0 0 0 0 0 0 0]);
qdf = deg2rad([0 -130 0 200 0 300 0]);

q = polynomial_trajectory(qi, qf, t, qdi, qdf);
move(s(1:7), q, dt);


% pause(0.26)

% Release the ball
move(s(8), 0, dt);