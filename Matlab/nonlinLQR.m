Q = diag([1e-2 1e-2 1e-2 1e-2 1 1 1 1]);
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
param.C= C;
param.G = G;
param.U=U;
     
A= [zeros(4) , eye(4) ; zeros(4,8)];


% B = [zeros(4,1); 1/m1 ; 1/m2 ; 1 ; 1];
  B = [zeros(4,4); diag([1/m1 ; 1/m2 ; 1 ; 1])];
    
% A_ctrl = [A(2,2),A(2,4);A(4,2),A(4,4)];
% 
% B_ctrl = [B(2);B(4)];
% 
% Q_ctrl = diag([1e3 1e5]);
% R_ctrl = 1;

[K,P,E]=lqr(A,B,Q,R)  %k=full state feedback gain.....P= solution to algebric ricaati equation....E= Eigrn values

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
% for i = 1:11
% K(1) = 0
% K(3) = 0
[tx,xx] = ode45(@(t,q,u) LQR(t,q,K,u0),tspan,q0); 
    
%     XX = [XX,x0];
%     UU = [UU,u];
    
% end

%%plotting
figure(1)
plot(tx,xx)
legend('r1','r2','theta','z','dr1','dr2','dtheta','dz')
% figure(2)
% plot(t,XX(1,:),t,XX(2,:),t,XX(3,:),t,XX(4,:))
  
function dx = LQR(t,q,K,u0)

% A= [zeros(4) , eye(4) ; zeros(4,8)];
% 
% 
% B = [zeros(4,1); 1/m1 ; 1/m2 ; 1 ; 1];
global param
param.U = -K*q;
% u = -K*x*0;

% dx = A*x + B*u;

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





