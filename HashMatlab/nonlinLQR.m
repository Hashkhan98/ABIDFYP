Q = diag([1e-2 1e-2 1e-2 1e-2 1 1 1 1]);
R = diag([1]);

m1 = 1;
m2 = 3.6;
mbh = 1.4;
mbv = 5;
bvr = .015;
g = 9.81;
lb = 1.4;
Ib = 1/12 * mbh * lb^2 + mbh * (lb/2)^2 + 1/2 * mbv * bvr^2;
     
A= [zeros(4) , eye(4) ; zeros(4,8)];


B = [zeros(4,1); 1/m1 ; 1/m2 ; 1 ; 1];
  
    
% A_ctrl = [A(2,2),A(2,4);A(4,2),A(4,4)];
% 
% B_ctrl = [B(2);B(4)];
% 
% Q_ctrl = diag([1e3 1e5]);
% R_ctrl = 1;

[K,P,E]=lqr(A,B,Q,R)  %k=full state feedback gain.....P= solution to algebric ricaati equation....E= Eigrn values

%Eigen values
E = eig(A-B*K)

G = -K

%%
x0 = [0 0 -100 50]';
u0 = 0;

tspan = [0 1]; 
XX = [];
UU = [];
% for i = 1:11
% K(1) = 0
% K(3) = 0
[tx,xx] = ode45(@(t,x) LQR(t,x,K),tspan,x0); 
    
%     XX = [XX,x0];
%     UU = [UU,u];
    
% end

%%plotting
figure(1)
plot(tx,xx)
legend('zdot','thetadot','z','theta')
% figure(2)
% plot(t,XX(1,:),t,XX(2,:),t,XX(3,:),t,XX(4,:))
  






