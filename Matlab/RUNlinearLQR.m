Q = diag([1e4 1e4 1e4 1e4 1 1 1 1]);
R = diag([1]);

q0 = [    0.8000;
    0.2222;
         0;
    0.9000;
         0;
         0;
         0;
         0];
u0 = [0 0 0 0]';

% Const
m1 = 1;
m2 = 3.6;
mbh = 1.4;
mbv = 5;
bvr = .015;
g = 9.81;
lb = 1.4;
Ib = 1/12 * mbh * lb^2 + mbh * (lb/2)^2 + 1/2 * mbv * bvr^2; 

Fz = g*(m1+m2+mbh) +u0(4);

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


U = [F1,F2,Tau,Fz]';

M = diag([m1 m2  m1*r1^2+m2*r2^2+Ib m1+m2+mbh]);

C = [0 0 -m1*r1*w 0;
       0 0 -m2*r2*w 0;
       2*m1*r1*w   2*m2*r2*w 0 0;
       0 0 0 0];

G = [0;0;0;g*(m1+m2+mbh)];



%% Mq** + Cq* + G = T
%% q** = M^-1 (-Cq* - G + T)
% 
% ddq = inv(M) * (-C*dq - G + U)

%% sim
% q = [r1 r2 theta z]

global param
param.M = M;
param.C = C;
param.G = G;
param.U = U;
     
A= [zeros(4) , eye(4) ; zeros(4,8)];


% B = [zeros(4,1); 1/m1 ; 1/m2 ; 1 ; 1];
B = [zeros(4,4); diag([1/m1 ; 1/m2 ; 1 ; 1])];

C = [1,1,1,1,0,0,0,0];

[K,P,E]=lqr(A,B,Q,R);  %k=full state feedback gain.....P= solution to algebric ricaati equation....E= Eigrn values

N_ff = 1\(C/((- A + B * K))* B);

%Eigen values
% E = eig(A-B*K)

% G = -K

U = [F1,F2,Tau,Fz]';

M = diag([m1 m2  m1*r1^2+m2*r2^2+Ib m1+m2+mbh]);

C = [0 0 -m1*r1*w 0;
       0 0 -m2*r2*w 0;
       2*m1*r1*w   2*m2*r2*w 0 0;
       0 0 0 0];

G = [0;0;0;g*(m1+m2+mbh)];

global param
param.M = M;
param.C= C;
param.G = G;
param.U=U;

%%

tspan = [0 0.1]; 
XX = [];
UU = [];

simsteps = 10; 
simstep = 0.1;

qall = zeros(8,simsteps/simstep);
qall(:,1) = q0;

time = 1:simsteps/simstep;
% [tx,xx] = ode45(@(t,q) LQR(t,q,K),tspan,q0); 
    
tic
for i = 2:(simsteps/simstep)  
%     qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
    [tx,xx] = ode45(@(t,q) LQR(t,q,K,qgoal),tspan,qall(:,i-1));
    qall(:,i) = xx(end,:)';

end
toc

%     XX = [XX,x0];
%     UU = [UU,u];
    
% end

%% plotting
clf;
figure(1)
plot(time, qall(1:4,:))

hold on
plot(time,repmat(qgoal(1:4,:),1,simsteps/simstep),'r--')
hold off
legend('r1','r2','theta','z','des r1','des r2','des theta','des z')

figure(2)
plot(time, qall(5:8,:))

hold on
plot(time,repmat(qgoal(5:8,:),1,simsteps/simstep),'r--')
hold off
legend('dr1','dr2','dtheta','dz','des dr1','des dr2','des dtheta','des dz')


figure(3)
plot(time,-K*(qall-repmat(qgoal,1,simsteps/simstep)))
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
param.U = -K*(q-qgoal);
% dx = A*x + B*u;
param.U = param.U + [0;0;0;g*(m1+m2+mbh)];

m1 = 1;
m2 = 3.6;
mbh = 1.4;
mbv = 5;
bvr = .015;
g = 9.81;
lb = 1.4;
Ib = 1/12 * mbh * lb^2 + mbh * (lb/2)^2 + 1/2 * mbv * bvr^2;

%%feedback linearisation
fx = [q(5:8);0;0;0;0];
gx = [0;0;0;0;(param.U(1) + m1*q(1)*q(7)^2)/m1;...
      (param.U(2) + m2*q(2)*q(7)^2)/m2;...
      (param.U(3)  - (2*m1*q(1)*q(5)*q(7) + 2*m2*q(2)*q(6)*q(7)))/(m1*q(1)^2 + m2*q(2)^2 + Ib);...
      (param.U(4))/(m1+m2+mbh) - g];
dx = fx + gx;

end





