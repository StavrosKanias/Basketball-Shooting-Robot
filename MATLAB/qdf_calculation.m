clear; 
clc;

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

% Generate the object robot
robot = SerialLink(L);
% Insert servo limitations to the robotic model
robot.qlim = ql;
% Name the robot
robot.name = "Ros";

dt = 0.000001;
% Select the final joint angle configuration of the shoot trajectory
q = deg2rad([0 -20 0 0 0 -8 0]);
[~,pd,quatd]=qconv(robot,q);
eta = quatd.s;
epsilon = quatd.v;
% Select the gain matrices
k0 = 10;
kp = 20;
ko = 20;
Kp = diag([kp kp kp]);
Ko = diag([ko ko ko]);
% Closed loop
K = [Kp diag([0 0 0]); diag([0 0 0]) Ko];
% Maximum number of iterations
E = zeros(1,7);
% Desired velocity
ue =  [1.2648 0 -0.2688 0 6.1 0]';
i=1;
% Update Jacobian, current pos and current orientation
[J,pos,quat]=qconv(robot,q);
% Update the desired pose
pd(:,i+1)= pd(:,i) + dt * ue(1:3);
eta(i+1) = eta(:,i) - dt * (epsilon(i,:) * ue(4:6))/2;
epsilon(i+1,:) = epsilon(i,:) + (dt * (eta(i) * eye(3) - skew(epsilon(i,:))) * ue(4:6)/2)';
quatd = UnitQuaternion(eta(i+1), epsilon(i+1,:));
% Define the tracking errors
ep = pd(:,i+1) - pos;
eo = (quat.s*quatd.v - quatd.s* quat.v)' - (skew(quatd.v)*quat.v');
e(:,i+1) = [ep;eo];
% Manipulability cost function
for j=1:7
    Ej = E;
    Ej(j) = 1;
    qd0(j,i+1) = k0*(robot.maniplty(q(i,:)+dt*Ej) - robot.maniplty(q(i,:)-dt*Ej));
end 
pseudoJ = pinv(J);
nul = (eye(7) - pseudoJ*J) * (qd0(:,i+1));
qdot = (pseudoJ*(ue+K*e(:,i+1)) + 1*nul)';
qdot = rad2deg(qdot)
