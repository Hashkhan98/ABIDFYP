function [c,ceq] = nonlcon(U,qgoal, q0,H,tspan)

global m1 m2

tolerance = 0.05;

ceq = [];
c=[q0(1)*m1 - q0(2)*m2 ];
end