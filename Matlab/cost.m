function c = cost(U,qgoal, q0,H,tspan)

ctemp = 0; 

Q = diag([50,50,50,50,1,1,1,1]);

R = diag([1 1 1 1]);

q1 = repmat(q0,1,H+1);

for i = 1:H
    
    q1(:,i+1)=massmatrix(q1(:,i),U(:,i),tspan);
    
%    qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
    
    ctemp = ctemp + (1/2)*(qgoal - q1(:,i))'*Q*(qgoal - q1(:,i))*(i^10) + 1/2*(U(:,i)'*R*U(:,i));
end

c = ctemp;

end