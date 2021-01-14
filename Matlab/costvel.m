function c = costvel(U,qstar,qstardot,qstardotdot, q0,H,tspan,simstep,simtime)

ctemp = 0; 

Q = diag([2e5 2e5 1e5 0.1e5 2e1 2e1 1e1 0.1e1]);

R = diag([1 1 1 1]);

q1 = repmat(q0(:,1),1,H+1);

for i = 1:H
    q1(:,i+1)= massmatrix(q1(:,i),U(:,i),tspan);
end

if simstep + H >= length(simtime)
    qstar = [qstar,repmat(qstar(:,end),1,H)]; 
    qstardot = [qstardot,repmat(qstardot(:,end),1,H)]; 
end

for i = 1:H
%    qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
    
ctemp = ctemp + (([qstar(:,simstep -1 + i);qstardot(:,simstep -1 + i)] - q1(:,i))'*Q*([qstar(:,simstep -1 + i);qstardot(:,simstep -1 + i)] - q1(:,i)))*(i^2);

end
c = ctemp;
end
% 
% for i = 1:H
%     
%     q1(:,i+1)= massmatrix(q1(:,i),U(:,i),tspan);
%     q1dot = gradient(q1);
%     
% %    qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
%     
%     ctemp = ctemp + (1/2)*(q1(:,i) - [qstar(:,i);qstardot(:,i)])'*Q*(q1(:,i) - [qstar(:,i);qstardot(:,i)])*(i^2); ...
% %             + (1/2)*([qstardot(:,i);qstardotdot(:,i)] - q1dot(:,i))'*Q*([qstardot(:,i);qstardotdot(:,i)] - q1dot(:,i))*(i^8); 
% %             + 1/2*(U(:,i)'*R*U(:,i));
% end
% 
% c = ctemp;
% 
% end