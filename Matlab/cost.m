function c = cost(U,qgoal, q0,H,tspan)

ctemp = 0; 

Q = diag([10,10,10,10,1,1,1,1]);

q1 = repmat(q0,1,H+1);

for i = 1:H
    q1(:,i+1)=massmatrix(q1(:,i),U(:,i),tspan);
   
    
    ctemp = ctemp + (qgoal - q1(:,i))'*Q*(qgoal - q1(:,i))*(i^2);
end

c = ctemp;

end