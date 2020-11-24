function out = cost(q, egoal,pose)
global m g
global tau


%cost for egoal
[AL1, A12, A23, A34, ABL, rGe,AG4] = FKM(q,pose);
e = [rGe(:,5,1);rGe(:,5,2);rGe(:,5,3);rGe(:,5,4);rGe(:,5,5);rGe(:,5,6)];
egoalCol = egoal(:);

%cost for 
% Calculate forces
rB4 = rGe - pose(1:3)';
I = eye(3);
C = [ I,I,I,I,I,I; skew(rB4(:,5,1)) skew(rB4(:,5,2)) skew(rB4(:,5,3)) skew(rB4(:,5,4)) ...
    skew(rB4(:,5,5)) skew(rB4(:,5,6))];
M = eulerRotation(pose(4:6))*([0 0 m*g]');
W = [M' 0 0 0]';
F = pinv(C)*W;
% F = C' * (inv(C*C'))*W;

Force(:,1) = F(1:3);
Force(:,2) = F(4:6);
Force(:,3) = F(7:9);
Force(:,4) = F(10:12);
Force(:,5) = F(13:15);
Force(:,6) = F(16:18);

AJ = analyticalJacobian(AL1, A12, A23, A34);
for j=1:6
for    i =1:4
tau(i,j) = AJ(:,i,j)'*Force(:,j);
end
end
maxtau(i,j) = max(abs(AJ(:,i,j)'*Force(:,j)),[],'all');

% out = [egoalCol - e(:)]'*1000*eye(size(max(egoalCol)))*[egoalCol - e(:)];

out = tau(:)'*eye(size(max(tau(:))))*tau(:)+maxtau(:)'*5*eye(size(max(maxtau(:))))*maxtau(:);
% out  = 0;

end