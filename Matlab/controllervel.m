function u = controllervel(q0, qstar,qstardot,qstardotdot, H,tspan,linflag,simstep,simtime)

persistent U0
if isempty(U0)
U0 = zeros(4, H);
end
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-3 ,-3 , -5, -10 + 58]';
ub = [3 ,3 ,5 ,10 + 58]';
lbmat = repmat(lb,1,H);
ubmat=repmat(ub,1,H);

nonlin = [];
% nonlin = @(U) nonlcon(U,qstar, q0,H,tspan);

options = optimoptions('fmincon','Display','off','Algorithm','sqp','MaxFunEvals',6000,'ConstraintTolerance',1e-3);
utemp = fmincon(@(U) costvel(U,qstar,qstardot,qstardotdot, q0,H,tspan,simstep,simtime) ,U0,A,b,Aeq,beq,lbmat,ubmat,nonlin,options);
% @(U) nonlcon(U,qgoal, q0,H,tspan)
U0=utemp;
u = utemp(:,1);
end