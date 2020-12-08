%% Simulate and plot the Tower Crane using LQR and a Nonlinear Model
clear;close all;

%% Declare Constants
Parameters;

%%Weights for Q and R for each state
Q = diag([1e3 1e5 50 5e3 10 10 10 1000]);
R = diag([1]);

q0 = [0.8000; 0.2222; 0; 0.9000; 0; 0; 0; 0];
u0 = [0 0 0 0]';

Fz = g*(m1+m2+mbh) +u0(4);

qgoal = [0.7,0.8*m1/m2,20*pi/180,0.8 ,0,0,0,0]';
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
     
A = [zeros(4) , eye(4) ; zeros(4,8)];

B = [zeros(4,4); diag([1/m1 ; 1/m2 ; 1 ; 1])];

[K,P,E]=lqr(A,B,Q,R);  %k=full state feedback gain.....P= solution to algebric ricaati equation....E= Eigrn values

%feedforward gain, may or may not be used
% C = [1,1,1,1,0,0,0,0];
% N_ff = 1\(C/((- A + B * K))* B);


%%
 
XX = [];
UU = [];

simtime = 5; % Length of simulation in seconds
T = 0.01; % 
simstep = [0 T];

qall = zeros(8,simtime/T);
qall(:,1) = q0;

time = 0:T:simtime;
% [tx,xx] = ode45(@(t,q) LQR(t,q,K),tspan,q0); 
    
%% Trajectory stuff

% Generate waypoint intercept times and position/velocity/acceleration data
[tWpts, XWpts] = generateWaypoints(simtime,T,q0,qgoal);

% Compute polynomial spline coefficients
param.traj = trajectoryDesign(tWpts, XWpts);

%%
tic
for i = 1:(simtime/T)  
%     qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
    [tx,xx] = ode45(@(t,q) LQR(t,q,K,qgoal),simstep,qall(:,i));
    qall(:,i+1) = xx(end,:)';

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

figure(2)
plot(time, qall(5:8,:))

hold on
plot(time,repmat(qgoal(5:8,:),1,length(time)),'r--')
hold off
legend('dr1','dr2','dtheta','dz','des dr1','des dr2','des dtheta','des dz')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)

figure(3)
plot(time,-K*(qall-repmat(qgoal,1,length(time))))
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',11)
% figure(2)
% plot(t,XX(1,:),t,XX(2,:),t,XX(3,:),t,XX(4,:))
  
function dx = LQR(t,q,K,qgoal)

% A= [zeros(4) , eye(4) ; zeros(4,8)];
% 
% 
% B = [zeros(4,1); 1/m1 ; 1/m2 ; 1 ; 1];
global param
Parameters;
% param.U = -K*q;
%%ref tracking
param.U = -K*(q - qgoal);

param.U = param.U + [0;0;0;g*(m1+m2+mbh)];

ddq = inv(param.M) * (-param.C*q(5:8) - param.G + param.U);

dx = [q(5:8);ddq];

%% trajectory stuff
trajectoryOutputHelper(t);
%%

end





