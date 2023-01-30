clear; 
clc;

% SIMULATE ARM TRAJECTORY

% DEFINE DH PARAMETERS
a = [ 0, 0, 0, 0, 0, 0, 0 ];
alpha= [ 90, -90, -90, 90, -90, 90, 0];
alpha = deg2rad(alpha);
d1 = 0.115; d3 = 0.115; d5 = 0.085; d7 = 0.23;
d =  [ d1, 0, d3, 0, d5, 0, d7];

ql = deg2rad([-90,90;-45,90;-135,45;-120,120;-135,125;-30,130;-135,125]);

n = 7;
% create robot's Links
for i = 1:n
    L(i) = Link( [0 d(i) a(i) alpha(i)] ) ;
end

% generate the object robot
robot = SerialLink(L);
%Insert servo limitations to the robotic model
robot.qlim = ql;
%name the robot
robot.name = "Ros";

dt = 0.005;
t = 0:dt:2;
% Could use inverse kinematice to find the qi and qf

qi = deg2rad([0 15 0 -30 0 -40 0]);
qf = deg2rad([0 -20 0 0 0 -8 0]);
qdi = deg2rad([0 0 0 0 0 0 0]);
% Best solution in practice
qdf = deg2rad([0 -130 0 200 0 300 0]);
% Recommended solution from inverse differential kinematics
% qdf = deg2rad([0.0000   88.9855    0.0000  136.0627    0.0000  302.4026   -0.0000]);

[q,qd,qdd] = polynomial_trajectory(qi, qf, t, qdi, qdf);
qfe = q(:,end);
qdfe = qd(:,end);
qddfe = qdd(:,end);

% Create the 3D simulation for the robot

figure;
robot.plot(q,'trail', 'k--','delay',dt,'lightpos', [1 1 1]);

% Plot the position, velocity and acceleration of the joints

figure;

% Position
subplot(3,1,1);
for i = 1:n
    plot(t, q(:,i), 'LineWidth', 1.5);
    hold on
end
legend('Joint 1','Joint 2', 'Joint 3', 'Joint 4','Joint 5', 'Joint 6', 'Joint 7');

title('Position of joints');
xlabel('t (seconds)');
xlim([0 max(t)]);
ylabel('Position (rad)'); 

% Velocity

subplot(3,1,2);
for i = 1:n
    plot(t, qd(:,i), 'LineWidth', 1.5);
    hold on
end
legend('Joint 1','Joint 2', 'Joint 3', 'Joint 4','Joint 5', 'Joint 6', 'Joint 7');

title('Velocity of joints');
xlabel('t (seconds)');
xlim([0 max(t)]);
ylabel('Velocity (rad/s)'); 

% Acceleration

subplot(3,1,3);
for i = 1:n
    plot(t, qdd(:,i), 'LineWidth', 1.5);
    hold on
end
legend('Joint 1','Joint 2', 'Joint 3', 'Joint 4','Joint 5', 'Joint 6', 'Joint 7');

title('Acceleration of joints');
xlabel('t (seconds)');
xlim([0 max(t)]);
ylabel('Acceleration (rad)'); 
