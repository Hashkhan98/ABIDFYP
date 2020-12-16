function c = costvel(U,qstar,qstardot,qstardotdot, q0,H,tspan)

ctemp = 0; 

Q = diag([1,1,1,1,0.1,0.1,0.1,0.1])*10000;

R = diag([1 1 1 1]);

q1 = repmat(q0(:,1),1,H+1);


for i = 1:H
    
    q1(:,i+1)= massmatrix(q1(:,i),U(:,i),tspan);
    q1dot = gradient(q1);
%    qall(:,i)=massmatrix(qall(:,i-1),u(:,i-1),tspan)';
    
ctemp = ctemp + (([qstar(:,i);qstardot(:,i)] - q1(:,i))'*Q*([qstar(:,i);qstardot(:,i)] - q1(:,i))); 

% ctemp = ctemp + (([qstar(:,i);qstardot(:,i)] - q1(:,i))'*Q*([qstar(:,i);qstardot(:,i)] - q1(:,i)))*(i^2) + 1/2*(U(:,i)'*R*U(:,i)) + ([qstardot(:,i);qstardotdot(:,i)] - q1dot(:,i))'*Q*([qstardot(:,i);qstardotdot(:,i)] - q1dot(:,i))*(i^2);
% (q1 - [qstar(:,i:H);qstardot(:,i:H)])
%             + ([qstardot(:,i);qstardotdot(:,i)] - q1dot(:,i))'*Q*([qstardot(:,i);qstardotdot(:,i)] - q1dot(:,i))*(i^8); 

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