function ctemp = costvel(U,qstar, q,H,tspan,simstep,Q,R)

ctemp = 0; %Initialise nil cost

% q = repmat(q0(:,1),1,H+1);        %Pre-allocate size for faster execution
% q = [q0,q0,q0,q0];          %Pre-allocate size for faster execution assuming H = 3

for i = 1:H

    q(:,i+1)= massmatrix(q(:,i),U(:,i),tspan);      %Compute dx = fx + gu based on current x and u over horizon H 
    ctemp = ctemp + ((qstar(:,simstep -1 + i) - q(:,i))'*Q*(qstar(:,simstep -1 + i) - q(:,i))) + U(:,i)'*R*U(:,i);
%     ctemp = ctemp + ((qstar(:,simstep -1 + i) - q(:,i))'*Q*(qstar(:,simstep -1 + i) - q(:,i)));

end

end
