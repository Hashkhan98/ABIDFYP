function [pos,vel,acc] = IKM(posvec,velvec,accvec)

Parameters_DH
m1 = 1;
m2 = 3.6;

x = -posvec(1);
y = -posvec(2);
z = posvec(3);

dx = -velvec(1);
dy = -velvec(2);
dz = velvec(3);

ddx = -accvec(1);
ddy = -accvec(2);
ddz = accvec(3);

q = atan2(x,-y);
r1 = sqrt(x^2+y^2);
r2 = (-0.08+.5+r1)/3.6;

dr1 = (x*dx + y*dy)/sqrt(x^2+y^2);
dr2 = (-0.08+.5+dr1)/3.6;
dq = (x*dy - dx*y)/(x^2+y^2);

ddr1 = ddx - x*dq^2;
ddr2 = (-0.08+.5+ddr1)/3.6;
ddq = x*sqrt(ddx^2 + ddy^2) + 2*dr1*dq;

% FKM([r1,r2,q,z]);

pos = [r1,r2,q,z]';
vel = [dr1,dr2,dq,dz]';
acc = [ddr1,ddr2,ddq,ddz]';
