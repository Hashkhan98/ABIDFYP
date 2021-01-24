function ctemp = costvel(U,qstar, q,H,tspan,simstep,Q,R)

ctemp = 0; %Initialise nil cost

for i = 1:H

    q(:,i+1)= massmatrix(q(:,i),U(:,i),tspan);      %Compute dx = fx + gu based on current x and u over horizon H 
    ctemp = ctemp + ((qstar(:,simstep -1 + i) - q(:,i))'*Q*(qstar(:,simstep -1 + i) - q(:,i))) + U(:,i)'*R*U(:,i);

end

end
