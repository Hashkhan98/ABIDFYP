 function [A01,A12_c_m1,A12_c_m2, A12_m1, A23_m1,A12_m2,A23_m2,A23_c_m1,A23_c_m2] = FKM(pose)

%%  define functions
% DH convention homogeneous transformation
% A_n = Rotation_z(theta)*Translation_z(d)*Rotation_x(alpha)*Translation_x(a)
Parameters_DH;
global   m1_a1 m1_a2 m1_a3 ...
            alpha_m1_1 alpha_m1_2 alpha_m1_3 ...
            m1 m2 g...
            m1_d1...
            lb_m1 lb_m2...
            mb1 mb2...
            m2_a1 m2_a2 m2_a3 ...
            alpha_m2_1 alpha_m2_2 alpha_m2_3 ...
            m2_d1
     
        
r1 = pose(1);
r2 = pose(2);
q  = pose(3);
z  = pose(4);

% center of gravity of beams
COG_Beam_m1 = (m1*r1+mb1*lb_m1/2)/(m1+mb1);
COG_Beam_m2 = (m1*r2+mb2*lb_m2/2)/m2+mb2;

% Denavit Hartenberg Matrix        
T = @(q,d,a,alpha) [cos(q)        -sin(q)*cos(alpha)           sin(q)*sin(alpha)       a*cos(q) ;
                              sin(q)         cos(q)*cos(alpha)          -cos(q)*sin(alpha)      a*sin(q);
                                    0          sin(alpha)                      cos(alpha)                 d;
                                    0          0                                   0                              1    ];
                                
%% calculate transofrmation matricies
A01 = T(q,m1_d1,m1_a1,alpha_m1_1); % same for all masses

% Transformation Matrices for m1
A12_m1 = T(0,z,m1_a2,alpha_m1_2);
A23_m1 = T(0,r1,m1_a3,alpha_m1_3);

% Transformation Matrices for beam's COM of m1 
A12_c_m1 = T(0,.5,m1_a2,alpha_m1_2);

A23_c_m1 = T(0,COG_Beam_m1,m1_a3,alpha_m1_3);

% Transformation Matrices for m2
A12_m2 = T(0,z,m2_a2,alpha_m2_2)*[eul2rotm([0,0,pi],'XYZ') [0;0;0]; 0 0 0 1];
A23_m2 = T(0,r2,m2_a3,alpha_m2_3);

% Transformation Matrices for beam's COM of m1 
A12_c_m2 = T(0,.5,m2_a2,alpha_m2_2)*[eul2rotm([0,0,pi],'XYZ') [0;0;0]; 0 0 0 1];

A23_c_m2 = T(0,COG_Beam_m2,m2_a3,alpha_m2_3);

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

%% Plotting
figure(3)
clf;
fig=plot3([0,0],[0,0], [0, 1], 'b-','LineWidth',6,'MarkerSize',4,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);
hold on

plot3([rB_mc1(1), rB_beam_m1(1)],[rB_mc1(2), rB_beam_m1(2)], [rB_mc1(3), rB_beam_m1(3)], ...
    'b-','LineWidth',6,'MarkerSize',4,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);

plot3([rB_mc1(1), rB_beam_m2(1)],[rB_mc1(2), rB_beam_m2(2)], [rB_mc1(3), rB_beam_m2(3)],...
    'b-','LineWidth',6,'MarkerSize',4,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);

plot3([rB_m1(1)],[rB_m1(2)], [rB_m1(3)], 'rs','LineWidth',6,'MarkerSize',4,'MarkerFaceColor','k','MarkerEdgeColor','k');
plot3([rB_m2(1)],[rB_m2(2)], [rB_m2(3)], 'rs','LineWidth',6,'MarkerSize',4*3,'MarkerFaceColor','k','MarkerEdgeColor','k');
plot3(0,0, rB_m2(3), 'ro','LineWidth',6,'MarkerSize',4*3,'MarkerFaceColor','k','MarkerEdgeColor','k');

drawFrame(T01,fig)
drawFrame(T03,fig)
drawFrame(T04,fig)

[x1 y1] = meshgrid(-1:0.1:1); % Generate x and y data
z1 = zeros(size(x1, 1)); % Generate z data
surf(x1, y1, z1) % Plot the surface

axis([-1 1 -1 1 0 1]);
title('Pose of the Robot')

xlabel('x','Fontsize',20);
ylabel('y','Fontsize',20);
zlabel('z','Fontsize',20);
set(gcf,'color','w');
hold off
end

