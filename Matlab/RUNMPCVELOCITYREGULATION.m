%% Simulate and plot the Tower Crane using MPC and a Nonlinear Model
clear;close all;

%% Declare Constants
Parameters;

simtime = 10; % Length of simulation in seconds
T = 0.01; % 
simstep = [0 T];

time = 0:T:simtime;

%% Trajectory 
wpts = [-0.5 +0.5 0.5 -0.5;
        -0.5 -0.5 0.5 0.5;
        0.4 0.4 0.4 0.4];

maxvel = 0.25;
[qstar,qstardot,qstardotdot,t] = TrajectoryGenerationNEW(wpts,maxvel); 

time = t; 

%%Weights for Q and R for each stat     e
Q = diag([2e5 2e5 1e5 0.1e5 2e3 2e3 1e3 0.1e3]);
% Q = diag([1 1 1 1 1 1 1 1]);
R = diag(1);

q0 = [qstar(:,1);qstardot(:,1)];
u0 = [0 0 0 0]';

% qgoal = [0.3,0.3*m1/m2,90*pi/180,0.4 ,0,0,0,0]';
% q = [r1 r2 theta z]
r1 = q0(1);
r2 = q0(2);
theta = q0(3);
z = q0(4);

dr1 = q0(5);
dr2 = q0(6);
w = q0(7);
dz = q0(8);

F1 = u0(1);
F2 = u0(2);
Tau = u0(3);
% Fz = g*(m1+m2+mbh) +u0(4);
Fz = u0(4);

U = [F1,F2,Tau,Fz]';

%% Define Mass, Coriolis and Gravity Matrix

% q = [r1 r2 theta z dr1 dr2 dtheta dz]
%%Mass Matrix
M = diag([m1 m2  m1*r1^2+m2*r2^2+Ib m1+m2+mbh]); 

%%Coriolis Matrix
C = [0 0 -m1*r1*w 0;
       0 0 -m2*r2*w 0;
       2*m1*r1*w   2*m2*r2*w 0 0;
       0 0 0 0];
%%Gravity Matrix
G = [0;0;0;g*(m1+m2+mbh)];

global param
param.M = M;
param.C = C;
param.G = G;
param.U = U;

param.A = [zeros(4) , eye(4) ; zeros(4,8)];

param.B = [zeros(4,4); diag([1/m1 ; 1/m2 ; 1 ; 1])];

[K,P,E]=lqr(param.A,param.B,Q,R);  %k=full state feedback gain.....P= solution to algebric ricaati equation....E= Eigrn values

%feedforward gain, may or may not be used
% C = [1,1,1,1,0,0,0,0];
% N_ff = 1\(C/((- A + B * K))* B);


%%
H = 3;

u = zeros(4,H);
u(:,1) = [F1,F2,Tau,Fz]';
qall = zeros(8,H);
qall(:,1) = q0;

%% sim
tic
for i = 1:length(time)-1

u(:,i) = controllervel(qall(:, i) + rand(8,1)*0.001 - rand(8,1)*0.001, qstar,qstardot,qstardotdot, H,simstep,0,i,time);

qall(:,i+1)= massmatrix(qall(:,i),u(:,i),simstep)';
end
toc
%% plotting
clf;
fig = figure(1);
% left_color = [.1 .1 0];
% right_color = [0 .5 .5];

plot(time, qall([1,2,4],:),'LineWidth',3)
ylabel('Displacement (m)')
hold on
plot(time,qstar([1,2,4],:),'r--','LineWidth',2)

yyaxis right
ax = gca;
ax.YColor = [20 155 20]/256;
plot(time, qall(3,:),'color',[20 155 20]/256,'LineWidth',3);
ylabel('Displacement (Rad)')
plot(time,qstar(3,:),'r--','LineWidth',2)
hold off

legend('r1','r2','z','des r1','des r2','des theta','des z','theta')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
xaxis([0 length(time)*T])
title('Position tracking of desired trajectory')

%%

RMSE = sqrt(mean((qall(1,:)' - qstar(1,:)').^2))
% plot(time,RMSE)
%%
figure(2)
plot(time, qall([5,6,8],:),'LineWidth',3)
hold on
plot(time,qstardot([1,2,4],:),'r--','LineWidth',3)
ylabel('Displacement (m/s)')

yyaxis right
ax = gca;
ax.YColor = 'g';
plot(time, qall(7,:),'g','LineWidth',3);
ylabel('Displacement (Rad/s)')
plot(time,qstardot(3,:),'r--','LineWidth',2)

hold off
legend('dr1','dr2','dz','des dr1','des dr2','des dz','dtheta','des dtheta')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
xaxis([0 length(time)*T])
title('Velocity tracking of desired trajectory')
%%
figure(3)
subplot(2,1,1)
plot(time(1:length(time)-1),u(1:3,:))
legend('Fr1','Fr2','Ftheta')
xlabel('Time (s)')
xaxis([0 length(time)*T])
ylabel('Torque (N.m)')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)

subplot(2,1,2)
plot(time(1:length(time)-1),u(4,:))
legend('Fz')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
ylabel('Force (N.m)')
xaxis([0 length(time)*T])
sgtitle('Control Forces required to maintain trajectory','fontweight','bold','fontsize',22)
%%
figure(4)
plot(time,qall(1,:)*m1,'k--',time,-qall(2,:)*m2,'b--',time,repmat(0.42,1,length(time)),'c')
hold on
plot(time,-qall(1,:)*m1 + qall(2,:)*m2 - 0.5 + 0.08,'g-')
legend('Torque m1','Torque m2','Torque Beam','Net Torque')
title('Torques acting on the joint of horizontal beam')
ylabel('Torque (N.m)')
xlabel('Time (s)')

set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xaxis([0 length(time)*T])
% (-0.08+0.5+r1)/m2;
%%
for i = 1:10:length(time)
   plotRobot([qall(1,i),qall(2,i),qall(3,i),qall(4,i)]);
end
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
% legend('a','b','c','Robot','arm ','bleh ','f')