%% Simulate and plot the Tower Crane using LQR and a Linear/Nonlinear Model
clear all;close all;
%% Declare Constants
Parameters;

simtime = 10; % Length of simulation in seconds
T = 0.01; % 
simstep = [0 T];

time = 0:T:simtime;
%% Trajectory 

% r= 0:0.01:1;
% Theta = -pi:2*pi/100:pi;
% z =repmat(0.4,1,101);
% [x,y,z] = pol2cart(Theta,r,z);

    
maxvel = 0.125;
[qstar,qstardot,qstardotdot,t,wpts] = TrajectoryGenerationNEW(maxvel,T); 
    
time = t;
%%Weights for Q and R for each state
Q = diag([2e3 2e3 1e3 0.1e3 2e2 2e2 1e2 0.1e2]);
% Q = diag([1 1 1 1 1 1 1 1]);
R = 0.001;

q0 = [qstar(:,1);qstardot(:,1)];
u0 = [0 0 0 0]';

% qstar = [0.7,0.8*m1/m2,20*pi/180,0.8 ,0,0,0,0]';
% qstardot = [0.1,0.1,1*pi/180,0.1 ,0.1,0.1,0.1,0.1]';
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

param.CC = eye(4);

[K,P,E]=lqr(param.A,param.B,Q,R);  %k=full state feedback gain.....P= solution to algebric ricaati equation....E= Eigrn values

%feedforward gain, may or may not be used
% C = [1,1,1,1,0,0,0,0];
% N_ff = 1\(C/((- A + B * K))* B);

%% SIM

qall = zeros(8,length(time));
qall(:,1) = q0;

tic
for i = 1:length(time)-1  
%     qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
    [~,xx] = ode45(@(t,q) LQR(t,q,K,qstar,qstardot,qstardotdot,i),simstep,qall(:,i));
    qall(:,i+1) = xx(end,:)';

end
toc

%% plotting
clf;
fig = figure(1);

p([1:3]) =plot(time, qall([1,2,4],:),'LineWidth',3);
ylabel('$\mathbf{Position(m)}$','interpreter','latex')
hold on
plot(time,qstar([1,2,4],:),'r--','LineWidth',2);

yyaxis right
ax = gca;
ax.YColor = [20 155 20]/256;
p(4) = plot(time, qall(3,:),'color',[20 155 20]/256,'LineWidth',3);
ylabel('$\mathbf{Position(rad)}$','interpreter','latex')
p(5) = plot(time,qstar(3,:),'r--','LineWidth',2);
hold off

legend(p(1:5),{'Actual $r_{1}$','Actual $r_{2}$','Actual $z$','Actual $\theta$','Desired Trajectory'},'interpreter','latex','Location','northeastoutside')
% legend(p([1:4]),'r1','r 2','z','','des r2','des theta','des z','theta')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
xaxis([0 length(time)*T])
title('Position tracking of desired trajectory')

%%
fig = figure(2);
clf;
% left_color = [.1 .1 0];
% right_color = [0 .5 .5];

p([1:3]) =plot(time, qall([5,6,8],:),'LineWidth',3);
ylabel('$\mathbf{Velocity(\frac{m}{s}}$)','interpreter','latex')
hold on
plot(time,qstardot([1,2,4],:),'r--','LineWidth',2);

yyaxis right
ax = gca;
ax.YColor = [20 155 20]/256;
p(4) = plot(time, qall(7,:)*180/pi,'color',[20 155 20]/256,'LineWidth',3);
ylabel('\bf{Velocity($\mathbf{\frac{degrees}{s}}$)}','interpreter','latex')
p(5) = plot(time,qstardot(3,:)*180/pi,'r--','LineWidth',2);
hold off

legend(p(1:5),{'Actual $\dot{r_{1}}$','Actual $\dot{r_{2}}$','Actual $\dot{z}$','Actual $\dot{\theta}$','Desired Trajectory'},'interpreter','latex','Location','northeastoutside')
% legend(p([1:4]),'r1','r 2','z','','des r2','des theta','des z','theta')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
xaxis([0 length(time)*T])
title('Velocity tracking of desired trajectory')

%%
fig = figure(3);
clf;
% left_color = [.1 .1 0];
% right_color = [0 .5 .5];
a = [gradient(qall(5,:))',gradient(qall(6,:))',gradient(qall(7,:))',gradient(qall(8,:))']*100;
p([1:3]) =plot(time, a(:,[1,2,4]),'LineWidth',3);
ylabel('$\mathbf{Acceleration(\frac{m}{s^2}})$','interpreter','latex')
hold on
plot(time,qstardotdot([1,2,4],:),'r--','LineWidth',2);

yyaxis right
ax = gca;
ax.YColor = [20 155 20]/256;
p(4) = plot(time, a(:,3),'color',[20 155 20]/256,'LineWidth',3);
ylabel('$\mathbf{Acceleration(\frac{degrees}{s^2}})$','interpreter','latex')
p(5) = plot(time,qstardotdot(3,:),'r--','LineWidth',2);
hold off

legend(p(1:5),{'Actual $\dot{r_{1}}$','Actual $\dot{r_{2}}$','Actual $\dot{z}$','Actual $\dot{\theta}$','Desired Trajectory'},'interpreter','latex','Location','northeastoutside')
% legend(p([1:4]),'r1','r 2','z','','des r2','des theta','des z','theta')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
xaxis([0 length(time)*T])
title('Acceleration tracking of desired trajectory')

%%
figure(4)
subplot(2,2,1)
plot(time(1:end-1),param.Uplot(1,:))


xlabel('Time (s)')
xaxis([0 length(time)*T])
ylabel('Force (N)')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
title('$F_{r1}$','interpreter','latex')

subplot(2,2,2)
plot(time(1:end-1),param.Uplot(2,:))


set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xlabel('Time (s)')
ylabel('Force (N)')
title('$F_{r2}$','interpreter','latex')
xaxis([0 length(time)*T])
sgtitle('Control Forces required to maintain trajectory','fontweight','bold','fontsize',22)

subplot(2,2,3)
plot(time(1:end-1),param.Uplot(3,:))


xlabel('Time (s)')
xaxis([0 length(time)*T])
ylabel('Torque (N.m)')
title('$F_{\theta}$','interpreter','latex')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)

subplot(2,2,4)
plot(time(1:end-1),param.Uplot(4,:))

xlabel('Time (s)')
xaxis([0 length(time)*T])
ylabel('Force (N)')
title('$F_z$','interpreter','latex')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)

%%
figure(5)
plot(time,qall(1,:)*m1,'r--',time,-qall(2,:)*m2,'b--',time,repmat(0.42,1,length(time)),'c')
hold on
plot(time,-qall(1,:)*m1 + qall(2,:)*m2 - 0.5 + 0.08,'g-')
legend('Torque m1','Torque m2','Torque Beam','Net Torque','interpreter','latex','Location','northeastoutside')
title('Torques acting on the joint of horizontal beam')
ylabel('Torque (N.m)')
xlabel('Time (s)')

set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
xaxis([0 length(time)*T])

%%
[x,y,z] = pol2cart(qall(3,:),qall(1,:),qall(4,:)+0.5);
[xstar,ystar,zstar] = pol2cart(qstar(3,:),qstar(1,:),qstar(4,:)+0.5);
figure(7)
clf
subplot(2,2,1)
plot3(xstar,ystar,zstar,'r--','LineWidth',2)
hold on
plot3(x,y,z,'b-')
axis([-0.2 0.7 -0.5 0.4 0.5 1.5]);

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
title('Trajectory Tracking of End effector')
view(90,90,90)


%%
% clf;
% figure(10)
% P = qall(3,:)*10;
% % plot(time,bleh)
% 
% Q = unwrap(P)/10;
% plot(time,Q,'r--',time,P/10,'b--')

%%
figure(8)
clf;
% plot(E)
scatter(real(E), imag(E),50,'filled');
hold on
plot(zeros(201,1)',[-10:0.1:10],'r--')
% xaxis([-( 10])
title('Closed-loop Eigen Values')
legend('Eigen Values' , 'Zero','interpreter','latex','Location','northeastoutside')
xlabel('Real')
ylabel('Imaginary')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
grid on

%%
% for i = 1:10:length(time)
%    plotRobot([qall(1,i),qall(2,i),qall(3,i),qall(4,i)+0.5],i == length(1:10:length(time)));
% end
%% 
function dx = LQR(~,q,K,qstar,qstardot,qstardotdot,i)

global param
Parameters;

% q = q + (rand(8,1) - rand(8,1))*0.001;
%ref tracking
U1 = -K*(q - [qstar(:,i);qstardot(:,i)]);

U2 = param.B \ (-param.A*[qstar(:,i);qstardot(:,i)] + [qstardot(:,i);qstardotdot(:,i)]);

% param.U = U1 + U2 + [0;0;0;g*(m1+m2+mbh)];  %gravity cancellation

param.U = U1 + U2;  %gravity cancellation

param.U(1) = param.U(1) - m1*q(1)*q(7)^2;
param.U(2) = param.U(2) - m2*q(2)*q(7)^2;
param.U(3) = param.U(3)*(m1*q(1)^2 + m2*q(2)^2 + Ib) + 2*m1*q(1)*q(5)*q(7) + 2*m2*q(2)*q(6)*q(7);
param.U(4) = (param.U(4)+g)*(m1+m2+mbh);

param.Uplot(:,i) = param.U;

%%feedback linearisation
fx = [q(5:8);0;0;0;0];
gx = [0;0;0;0;(param.U(1) + m1*q(1)*q(7)^2)/m1;...
      (param.U(2) + m2*q(2)*q(7)^2)/m2;...
      (param.U(3)  - (2*m1*q(1)*q(5)*q(7) + 2*m2*q(2)*q(6)*q(7)))/(m1*q(1)^2 + m2*q(2)^2 + Ib);...
      (param.U(4))/(m1+m2+mbh) - g];
dx = fx + gx;
% param.Uplot(:,i) = gx(5:8);
end






