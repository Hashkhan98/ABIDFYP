function [pos] = IKM(posvec,velvec,accvec)

Parameters_DH
% persistent q_0
% if q_0 == []
%     q_0 = 0;
% end

m1 = 1;
m2 = 3.6;

x = posvec(1);
y = posvec(2);
z = posvec(3);

% dx = -velvec(1);
% dy = -velvec(2);
% dz = velvec(3);
% 
% ddx = -accvec(1);
% ddy = -accvec(2);
% ddz = accvec(3);
[q,r1,z] = cart2pol(x,y,z);
% q = atan2(y,x);
% r1 = sqrt(x^2+y^2);
% r1=-r1;
r2 = (-0.08+.5+r1)/3.6;

% if q >= pi 
%     q = pi + mod(q,pi);
% else -q <= -pi 
%     q = -pi - mod(q,pi);
% end

% dr1 = (x*dx + y*dy)/sqrt(x^2+y^2);
% dr2 = (-0.08+.5+dr1)/3.6;
% dq = (x*dy - dx*y)/(x^2+y^2);
% 
% r = sqrt(x^2 + y^2);
% ddr = (x^3*ddx + x^2*dy^2 + x^2*y*dy + y^2*x*ddx - 2*x*y*dx*dy + y^2*dx^2 + y^3*ddy)/((x^2 + y^2)*sqrt(x^2+y^2));
% ddtheta = ((-y*ddx + x*ddy)*(x^2+y^2)* - (2*x*dx + 2*y*dy)*(x*dy - y*dx))/((x^2+y^2)^2); 
% 
% 
% ddr1 = ddr - r*dq^2;
% ddr2 = (-0.08+.5+ddr1)/3.6;
% ddq = r*ddtheta + 2*dr1*dq;

% FKM([r1,r2,q,z]);

pos = [r1,r2,q,z]';
% vel = [dr1,dr2,dq,dz]';
% acc = [ddr1,ddr2,ddq,ddz]';
