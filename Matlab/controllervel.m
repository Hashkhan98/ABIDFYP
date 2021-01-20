function u = controllervel(q0, qstar,qstardot,qstardotdot, H,tspan,linflag,simstep,simtime)

persistent U0
if isempty(U0)
U0 = zeros(4, H);
end

% nonlin = @(U) nonlcon(U,qstar, q0,H,tspan);


U0=utemp;
u = utemp(:,1);
end