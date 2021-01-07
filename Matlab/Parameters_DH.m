%% Denavit Hartenburg parameters
% Link lengths d (m) and link offsets a (m)
function Parameters_DH

%% Mass 1
global   m1_a1 m1_a2 m1_a3 ...
            alpha_m1_1 alpha_m1_2 alpha_m1_3 ...
            m1 m2 g...
            m1_d1...
            lb_m1 lb_m2...
            mb1 mb2

% DH Parameters
m1_a1 = 0;        m1_d1 = 0;
m1_a2 = 0;
m1_a3 = 0;


% link twist angles corresponding to DH parameter Alpha_n
alpha_m1_1 = 0;     alpha_m1_2 = pi/2;
alpha_m1_3 = 0;
%% Mass 2
global   m2_a1 m2_a2 m2_a3 ...
            alpha_m2_1 alpha_m2_2 alpha_m2_3 ...
            m2_d1

% DH Parameters
m2_a1 = 0;        m2_d1 = 0;
m2_a2 = 0;
m2_a3 = 0;


% link twist angles corresponding to DH parameter Alpha_n
alpha_m2_1 = 0;     alpha_m2_2 = -pi/2;
alpha_m2_3 = 0;
%% Beam Lengths
lb_m1 = 1;
lb_m2 = .4;
%% Mass Parameters
m1 = 1;
m2 = 3.6;
mb1 = 1;
mb2 = .4;
g = 9.81;
end