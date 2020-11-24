syms m1 m2 r1 r2 mb Ib theta z dtheta dr1 dr2 dz F1 F2 Tau Fz g

M = [m1 0 0 0;
    0 m2 0 0;
    0 0 m1*r1^2+m2*r2^2+Ib 0;
    0 0 0 m1+m2+mb];
V = (m1+m2+mb)*g*z;

dq = [dr1;dr2;dtheta;dz];

P = M*dq;

H =.5* P.'*inv(M)*P + V

U = [F1 F2 Tau Fz];

dP(1) = -.5 * diff((P.'*inv(M)*P),r1) - diff(V,r1) + U(1);
dP(2) = -.5 * diff((P.'*inv(M)*P),r2) - diff(V,r2) + U(2); 
dP(3) = -.5 * diff((P.'*inv(M)*P),theta) - diff(V,theta) + U(3);
dP(4) = -.5 * diff((P.'*inv(M)*P),z) - diff(V,z) + U(4);

dq

dP.'

