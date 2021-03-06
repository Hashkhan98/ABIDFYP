%% Simulate and plot the Tower Crane using MPC and a Nonlinear Model
clear;close all;

%% Declare Constants
Parameters;

q0 = [0.8000; 0.2222; 0; 0.9000; 0; 0; 0; 0];
u0 = [0 0 0 0]';

qgoal = [0.3,0.3*m1/m2,90*pi/180,0.4 ,0,0,0,0]';
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
     
%feedforward gain, may or may not be used
% C = [1,1,1,1,0,0,0,0];
% N_ff = 1\(C/((- A + B * K))* B);


%%
H = 3;

simtime = 5; % Length of simulation in seconds
T = 0.01; % 
simstep = [0 T];

time = 0:T:simtime;

u = zeros(4,H);
u(:,1) = [F1,F2,Tau,Fz]';
qall = zeros(8,H);
qall(:,1) = q0;

%% sim
tic
for i = 1:(simtime/T)

u(:,i) = controller(qall(:, i), qgoal, H,simstep,0);

qall(:,i+1)=massmatrix(qall(:,i),u(:,i),simstep)';
end
toc
%% plotting
clf;
figure(1)
plot(time, qall(1:4,:))

hold on
plot(time,repmat(qgoal(1:4,:),1,length(time)),'r--')
hold off
legend('r1','r2','theta','z','des r1','des r2','des theta','des z')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)
title('Set-Point Regulation of Position')


figure(2)
plot(time, qall(5:8,:))

hold on
plot(time,repmat(qgoal(5:8,:),1,length(time)),'r--')
hold off
legend('dr1','dr2','dtheta','dz','des dr1','des dr2','des dtheta','des dz')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)

figure(3)
plot(time(1:500),u)
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

