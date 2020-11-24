function [q1]=massmatrix(q0,u,tspan)

% Const
m1 = 1;
m2 = 3.6;
mbh = 1.4;
mbv = 5;
bvr = .015;
g = 9.81;
lb = 1.4;
Ib = 1/12 * mbh * lb^2 + mbh * (lb/2)^2 + 1/2 * mbv * bvr^2; 

Fz = g*(m1+m2+mbh) +u(4);

% q = [r1 r2 theta z]
r1 = q0(1);
r2 = q0(2);
theta = q0(3);
z = q0(4);

dr1 = q0(5);
dr2 = q0(6);
w = q0(7);
dz = q0(8);

F1 = u(1);
F2 = u(2);
Tau = u(3);


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
t=0;
xx = ode4(@(t,q) simulation1(t,q,param),tspan,q0);

q1 = xx(end, :)'; 


end

function dx =simulation1(t,q,param)

ddq = inv(param.M) * (-param.C*q(5:8) - param.G + param.U);

dx = [q(5:8);ddq];

end

