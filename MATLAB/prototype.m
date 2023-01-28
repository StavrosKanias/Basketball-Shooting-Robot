% % % clear; close all; clc;
% % % construct the object arduino
% % a = arduino('COM5');
% % % num of servos
% n = 7;
% dt = 0.005;
% % s1 = servo(a, 'D3',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s2 = servo(a, 'D4',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s3 = servo(a, 'D5',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s4 = servo(a, 'D6',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s5 = servo(a, 'D7',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s6 = servo(a, 'D8',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s7 = servo(a, 'D9',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
% % s8 = servo(a, 'D10',  'MinPulseDuration',500*10^-6, 'MaxPulseDuration', 4000*10^-6);
% % 
% % s = [s1;s2;s3;s4;s5;s6;s7;s8];
% % pause(1);
% 
% 
% writePosition(s(8), 0.95*(0 + 140) / (270));
% 
% qa = deg2rad([5,-2,-2,-12,0,18,5,0]);
% maxiter = 1;
% 
% % Use this after having calculated the joint angles q to move the servos
% % dt is the Euler integration step you chose (di corresponds to the sampling step - you might have to change it)
% di = 0.22/dt;
% %convert q from rads to degs
% theta = round(rad2deg(qa));
% % maxiter is the size of euler iterations
% angle = zeros(maxiter,n);
% %normalization to range [0,0.95]
% for j=1:n
%         angle(:,j) = 0.95*((theta(:,j) + 140) / (270));
% end
% 
% % write the normalized angles to each servo
% for i =1:di:maxiter
%     for j=1:n
%             %update the position of servos
%             writePosition(s(j), angle(i,j));
%             %position = readPosition(s(j));
%     end
% end
% 
% pause(3);
% 
% writePosition(s(8), 0.95*(-120 + 140) / (270));
% % 
% % 
% %%% YOUR CODE
% dq = [5,-2,-2,-12,0,18,5];
% dt = 0.005;
% t = 0:dt:2;
% qi = deg2rad([0,15,0,-5,0,-15,0] + dq);
% qf = deg2rad([0,5,0,15,0,20,0] + dq);
% qdi = deg2rad([0 0 0 0 0 0 0]);
% qdf = deg2rad([0 100 0 350 0 250 0]);
% 
% q = polynomial_trajectory(qi, qf, t, qdi, qdf);
% qa = q;
% 
% rad2deg(qa)
% maxiter = length(t);
% 
% % Use this after having calculated the joint angles q to move the servos
% % dt is the Euler integration step you chose (di corresponds to the sampling step - you might have to change it)
% di = 0.22/dt;
% %convert q from rads to degs
% theta = round(rad2deg(qa));
% % maxiter is the size of euler iterations
% angle = zeros(maxiter,n);
% %normalization to range [0,0.95]
% for j=1:n
%         angle(:,j) = 0.95*((theta(:,j) + 140) / (270));
% end
% 
% % write the normalized angles to each servo
% for i =1:di:maxiter
%     for j=1:n
%             %update the position of servos
%             writePosition(s(j), angle(i,j));
%             %position = readPosition(s(j));
%     end
% end
% pause(0.38)
% writePosition(s(8), 0.95*(0 + 140) / (270));

% 
