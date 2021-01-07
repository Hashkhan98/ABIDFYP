function [pos] = IKM(posvec,velvec,accvec)

Parameters_DH

m1 = 1;
m2 = 3.6;

x = posvec(1);
y = posvec(2);
z = posvec(3);
[q,r1,z] = cart2pol(x,y,z);
r2 = (-0.08+.5+r1)/3.6;

pos = [r1,r2,q,z]';

