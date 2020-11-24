function dx = LQR(t,x,K)

A= [zeros(4) , eye(4) ; zeros(4,8)];


B = [zeros(4,1); 1/m1 ; 1/m2 ; 1 ; 1];
  
u = -K*x;
u = -K*x*0;

dx = A*x + B*u;