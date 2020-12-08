%% init cond
% q = [r1 r2 theta z]

clear all;
Parameters_DH;
global   m1_a1 m1_a2 m1_a3 ...
            alpha_m1_1 alpha_m1_2 alpha_m1_3 ...
            m1 m2 g...
            m1_d1...
            lb_m1 lb_m2...
            mb1 mb2

r1 = 0.8;
r2 = r1*m1/m2;
theta = 0;
z = 0.9;

qgoal = [.3,.3*m1/m2,90*pi/180,.4 ,0,0,0,0]';

dr1 = 0;
dr2 = 0;
w = 0;
dz = 0;

F1 = 0;
F2 = 0;
Tau = 0;
Fz = 0;

q0h = [r1 r2 theta z]';
dq0h = [dr1 dr2 w dz]';
q0 = [q0h;dq0h] ;

u0 = 0;

K = ones(1,8)*0;

simtime = 5; 
simstep = 0.01;
H = 3; 

tspan = [0 simstep];

u = zeros(4,simtime);
u(:,1) = [F1,F2,Tau,Fz]';
qall = zeros(8,simtime);
qall(:,1) = q0;

time = 1:simtime/simstep;
%% sim
tic
for i = 2:(simtime/simstep)
    
qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
% qall(:,i)=feedbacklinearisation(qall(:,i-1),u(:,i-1),tspan)';

u(:,i) = controller(qall(:, i), qgoal, H,tspan,0);

end
toc
%% plotting
clf;
figure(1)
plot(time, qall(1:4,:))

hold on
plot(time,repmat(qgoal(1:4,:),1,simtime/simstep),'r--')
hold off
legend('r1','r2','theta','z','dr1','dr2','dtheta','dz')

figure(2)
plot(time,u)

% for i = 1:simtime/simstep
%     FKM([qall(1,i) qall(2,i) qall(3,i) qall(4,i)]);
%     i = i +1;
% end
% plot((1:(max(size(qall))))*simstep,qall(:,1:4))
% legend('r1','r2','theta','z')

