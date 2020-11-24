clc
clear all
%% SYMBOLIC VARIABLES
syms  r1     r2     theta      z  real    % variables
syms  r1_d   r2_d   theta_d    z_d   real % first derivative of variables % velocities
syms  r1_acc r2_acc theta_acc  z_acc real % Second derivative of variables % acceleration
syms m_l_h real                           % mass of horizontal beam
syms m_l_v real                           % mass of vertical beam
syms g m1 m2 l real                       % Constant parameters%
syms I_l_h_z real                         % mass moment of inertia of horizontal beam about z axis @ center of mass
syms I_bar_1 I_bar_2 real                 % about z-axis
syms R z1 z2 psi I1 I2 dz1 dz2 d_theta d_psi
q=[r1;r2;theta;z];        
q_dot=[r1_d;r2_d;theta_d;z_d];
q_acc=[r1_acc;r2_acc;theta_acc;z_acc];

%% POTENTIAL ENERGY
PE=m1*g*z1+m2*g*z2;

%% KINETIC ENERGY

% % VERTICAL BEAM
% k1= 0;
% 
% % horizontal beam
% f=(m_l_h/(l/3+l))*((R*theta_d)^2+z_d^2)*.5;
% k2=int(f,R,[-l,l/3]);
% % k2=(m_l_h*(7*l^2*theta_d^2 + 27*z_d^2))/54;
% 
% % m1
% k3=.5*m1*(z_d^2+r1_d^2+ (r1*theta_d)^2 ) + .5*I_bar_1*theta_d^2;
% 
% % m2
% k4=.5*m2*(z_d^2+r2_d^2+ (r2*theta_d)^2 ) + .5*I_bar_2*theta_d^2;


% TOTAL KINETIC ENERGY
K=.5*m1*diff(r1)^2+.5*m2*diff(r2)^2+.5*I1*d_theta^2+.5*I2*d_theta^2+.5*I1*d_psi^2+.5*I2*d_psi^2+.5*m1*dz1^2+.5*m2*dz2^2;

% Lagrangian
Lagrangian=K-PE

%% EQUATOINS

%% d(lagrangian)/d(x_dot)
for i=1:length(q)
    d_x_dot(i,1)=diff(Lagrangian,q_dot(i,1));
    d_x(i,1)=    diff(Lagrangian,q(i,1));
end
Q1=zeros(length(q),1);

for i=1:length(q)
    for j=1:length(q)
        
        M_mat(i,j)=diff(d_x_dot(i,1),q_dot(j,1)); % Inertia matrix
        C_mat(i,j)=diff(d_x_dot(i,1),q(j,1));     % Inertia matrix
    end
end


for i=1:length(q)
    
    G_vec(i,1)= - diff(Lagrangian,q(i,1));
    
end

M_mat
C_mat
G_vec
