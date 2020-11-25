function out = IKM(posvec)

Parameters_DH
m1 = 1;
m2 = 3;

x = -posvec(1);
y = -posvec(2);
z = posvec(3);

q = atan2(x,-y);
r1 = sqrt(x^2+y^2)
r2 = (-0.08+.5+r1)/3.6

FKM([r1,r2,q,z]);



% egoal= [x,y,z];

% persistent r2
% if isempty(r2) 
%     r2= [0];
% end
% 
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% lb = 0;
% ub = 1;
% options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunEvals',10000);
% r2_new = fmincon(@(r2) cost(r2,egoal) ,r2,A,b,Aeq,beq,lb,ub,@(r2) nonlcon(r2,egoal),options);
% r2=r2_new;
% end


out = [r1,r2,q,z];
