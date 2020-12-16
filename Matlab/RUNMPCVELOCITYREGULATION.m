%% Simulate and plot the Tower Crane using MPC and a Nonlinear Model
clear;close all;

%% Declare Constants
Parameters;

simtime = 10; % Length of simulation in seconds
T = 0.01; % 
simstep = [0 T];

time = 0:T:simtime;

%% Trajectory 
wpts = [-0.5 +0.5 0.5 -0.5 -0.5;
        -0.5 -0.5 0.5 0.5 -0.5;
        0.4 0.4 0.4 0.4 0.4];
% len = [1:5];
% 
% wpts = [0.25 + sin(len)*0.02;
%         0.25 + cos(len)*0.02;
%         ones(1,length(len))*0.4];
maxvel = 0.25;
[qstar,qstardot,qstardotdot] = TrajectoryGeneration(wpts,time,maxvel); 

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
for i = 1:(simtime/T)

u(:,i) = controllervel(qall(:, i), qstar,qstardot,qstardotdot, H,simstep,0,i,time);

qall(:,i+1)= massmatrix(qall(:,i),u(:,i),simstep)';
end
toc
%% plotting
clf;
figure(1)
plot(time, qall(1:4,:))

hold on
plot(time,qstar,'r--')
hold off
legend('r1','r2','theta','z','des r1','des r2','des theta','des z')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)
title('Set-Point Regulation of Position')


figure(2)
plot(time, qall(5:8,:))

hold on
plot(time,qstardot,'r--')
hold off
legend('dr1','dr2','dtheta','dz','des dr1','des dr2','des dtheta','des dz')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)

figure(3)
plot(time(1:length(time)-1),u)
legend('Fr1','Fr2','Ftheta','Fz')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)

figure(4)
plot(time,qall(1,:)*m1,'k--',time,qall(2,:)*m2,'b--')
hold on
plot(time,qall(1,:)*m1 - qall(2,:)*m2,'go',time,repmat(0.1,1,length(time)),'r-')

set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)
% for i = 1:simtime/simstep
%     FKM([qall(1,i) qall(2,i) qall(3,i) qall(4,i)]);
%     i = i +1;
% end
% plot((1:(max(size(qall))))*simstep,qall(:,1:4))
% legend('r1','r2','theta','z')

for i = 1:10:length(time)
   plotRobot([qall(1,i),qall(2,i),qall(3,i),qall(4,i)]);
end