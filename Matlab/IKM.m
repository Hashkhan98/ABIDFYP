function out = IKM(posvec)

Parameters_DH
m1 = 1;
m2 = 3;

x = -posvec(1);
y = -posvec(2);
z = posvec(3);

q = atan2(x,-y);
r1 = sqrt(x^2+y^2);
r2 = (-0.08+.5+r1)/3.6;

% FKM([r1,r2,q,z]);

out = [r1,r2,q,z]';
