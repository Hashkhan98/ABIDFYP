function [q1]=massmatrix(q0,u,tspan)

Parameters;

param.M = diag([m1 m2  m1*q0(1)^2+m2*q0(2)^2+Ib m1+m2+mbh]);

param.C = [0 0 -m1*q0(1)*q0(7) 0;
       0 0 -m2*q0(2)*q0(7) 0;
       2*m1*q0(1)*q0(7)   2*m2*q0(2)*q0(7) 0 0;
       0 0 0 0];

param.G = [0;0;0;g*(m1+m2+mbh)];

%% Mq** + Cq* + G = T
%% q** = M^-1 (-Cq* - G + T)
% 
% ddq = inv(M) * (-C*dq - G + U)

param.U = u;

[~,xx] = ode45(@(t,q) simulation1([],q,param),tspan,q0);

q1 = xx(end, :)'; 


end

function dx =simulation1(~,q,param)

ddq = param.M\(-param.C*q(5:8) - param.G + param.U);

dx = [q(5:8);ddq];

end

