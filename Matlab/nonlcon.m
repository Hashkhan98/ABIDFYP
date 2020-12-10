function [c,ceq] = nonlcon(U,qgoal, q0,H,tspan)

global m1 m2

ceq = [];
c = [abs(q0(1)*m1+0.5 - q0(2)*m2-0.08)-0.2 ];
% ; abs(q0(5:8))-0.5
c=[q0(1)*m1 - q0(2)*m2 ];
end