 function [] = plotRobot(pose,flag)
Parameters_DH;
r1 = pose(1);
r2 = pose(2);
q  = pose(3) - pi/2;
z  = pose(4);
% Denavit Hartenberg Matrix        
T = @(q,d,a,alpha) [cos(q) -sin(q)*cos(alpha) sin(q)*sin(alpha) a*cos(q) ;
                    sin(q) cos(q)*cos(alpha) -cos(q)*sin(alpha) a*sin(q);
                      0 sin(alpha) cos(alpha) d;
                      0 0 0 1];                             
%% calculate transofrmation matricies
A01 = T(q,m1_d1,m1_a1,alpha_m1_1); % same for all masses
% Transformation Matrices for m1
A12_m1 = T(0,z,m1_a2,alpha_m1_2);
A23_m1 = T(0,r1,m1_a3,alpha_m1_3);
% Transformation Matrices for m2
A12_m2 = T(0,z,m2_a2,alpha_m2_2)*[eul2rotm([0,0,pi],'XYZ') [0;0;0]; 0 0 0 1];
A23_m2 = T(0,r2,m2_a3,alpha_m2_3);
% Transformation from Ground to each joint also 3D matrices
T01 = A01;
T02_m1 = A01*A12_m1;
T03 = A01*A12_m1*A23_m1;
T04 = A01*A12_m2*A23_m2;
% Just to plot a fixed beam
T0_fixedbeam_m1 = A01*A12_m1*T(0,lb_m1,m1_a3,alpha_m1_3);
T0_fixedbeam_m2 = A01*A12_m2*T(0,lb_m2,m1_a3,alpha_m1_3);
%collecting little r's from Transformation matrices
rB_mc1 = T02_m1(1:3,4);
rB_m1= T03(1:3,4);
rB_m2= T04(1:3,4);
rB_beam_m1 = T0_fixedbeam_m1(1:3,4);
rB_beam_m2 = T0_fixedbeam_m2(1:3,4);