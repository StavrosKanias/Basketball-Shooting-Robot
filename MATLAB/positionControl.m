clear; 
clc;
%Arduino servo control

% Arduino communication
% delete(instrfind('Type', 'serial')); % Assure that no serial port is open anymore
% arduino = serialport('COM5',9600);   
% configureTerminator(arduino,"CR");
% arduino.Timeout = 40;
%num of servos 
n = 7;
% pause(2);  % give servo some time to initialize

% Commands:

a = [ 0, 0, 0, 0, 0, 0, 0 ];
alpha= [ 90, -90, 90, -90, -90, 90, 0];
alpha = deg2rad(alpha);
d1 = 0.115; d3 = 0.115; d5 = 0.085; d7 = 0.23;
d =  [ d1, 0, d3, 0, d5, 0, d7] ;
dq = [10,14,8,-5,10,25,12];
q0 = 90+dq;
%initial 
% q = [0,0,0,0,0,0,0];

% SIMULATION
ql = [-90,90;-45,90;-135,45;-120,120;-135,125;-30,130;-135,125];
ql = deg2rad(ql);

% create robot's Links
for i = 1:7
L(i) = Link( [0 d(i) a(i) alpha(i)] ) ;
end

% generate the object robot
robot = SerialLink(L);
%Insert servo limitations to the robotic model
robot.qlim = ql;
%name the robot
robot.name = "Ros";

dt = 0.1;
t = 0:dt:2;
qi = deg2rad([0,15,0,20,0,25,0]);
qf = deg2rad([0,4,0,6,0,8,0]);
qdi = deg2rad([0 0 0 0 0 0 0]);
qdf = deg2rad([0 -30 0 -40 0 -50 0]);
qi
qf
qdf
[q,qd,qdd] = polynomial_trajectory(qi, qf, t, qdi, qdf);
q
qd
qdd

figure;
robot.plot(q,'trail', 'k--','delay',dt,'lightpos', [1 1 1]);

%desired angles
% %q1
% q = [50,50,50,50,50,50,50];
% %q2
% q = [90,10,0,180,90,90,90];
% %q3
% q = [180,90,180,90,90,120,90];
% q4
% q = [90,90,90,90,90,90,90];
% 
% q = q + dq;
% % Tranfer Matrix
% T07 = fwkin(q,q0,d,alpha,a);
% T07

% q = [90,105,90,80,90,85,90];


% commands(1:n) = q;
% packet = sprintf('%s,' , string(commands));
% readline(arduino)
% write(arduino,packet,"string");

% %Upper and Lower angular bounds of motion
% aU = 180;
% % aL = 0;
% 
% stop = 1;
% % acceleration flag
% accel = [0 -10 0 12 0 15 0];
% qdot = [ 0 -100 0 120 0 150 0];
% qnew = ones(7);
% qold = [90,105,90,80,90,85,90];
% qold = qold + dq;
% dt = 1e-2;
% t = 0;
% while(stop)
%  t = t + dt;
%  % next position computation for each servo
%    %Euler integration
%    qdot = qdot + dt * accel;
%    qnew = qold + dt * qdot;
%    qold = qnew;
% 
%  commands(1:n) = qnew;
%  packet = sprintf('%s,' , string(commands));
%  readline(arduino)
%  write(arduino,packet,"string");
%  if qnew(2) < 95 || qnew(2) < 105
%      stop = 0;
%  end
% 
% end
