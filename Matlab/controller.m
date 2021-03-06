function u = controller(q0, qgoal, H,tspan,linflag)

persistent U0
if isempty(U0)
U0 = zeros(4, H);
end
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-5 ,-5 , -5, -5]';
ub = [5 ,5 ,5 ,5]';
lbmat = repmat(lb,1,H);
ubmat=repmat(ub,1,H);
if linflag
    nonlin = [];
else
   nonlin = @(U) nonlcon(U,qgoal, q0,H,tspan);
end
options = optimoptions('fmincon','Display','off','Algorithm','sqp','MaxFunEvals',6000,'ConstraintTolerance',1e-3);
utemp = fmincon(@(U) cost(U,qgoal, q0,H,tspan) ,U0,A,b,Aeq,beq,lbmat,ubmat,nonlin,options);
% @(U) nonlcon(U,qgoal, q0,H,tspan)
U0=utemp;
u = utemp(:,1);
end