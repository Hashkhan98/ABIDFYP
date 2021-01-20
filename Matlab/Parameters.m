%% Paramaters of the crane 

%% Constants

% global m1 m2 mbh mbv bvr g lb Ib M C G
m1 = 1;         %% Mass of the load 
m2 = 3.6;       %% Mass of the counterweight
mbh = 1.4;      %% Mass of the horizontal Beam
mbv = 5;        %% Mass of the vertical Beam
bvr = .015;     %% ????????
g = 9.81;       %% gravity
lb = 1.4;       %% Length of the horizontal beam
Ib = 1/12 * mbh * lb^2 + mbh * (lb/2)^2 + 1/2 * mbv * bvr^2; %% Inertia of the Horizontal beam
