function [t, X] = generateWaypoints(simtime,T,q0,qgoal)

% Waypoint intercept times
t0 = 0;                 % Waypoint 0 intercept time [s]
t1 = simtime;           % Waypoint 1 intercept time [s]
t = [t0 t1];

% Waypoint position, velocity, acceleration
%
% Each page of X has the following structure:
% X(:,:,i) = [q1, dq1, ddq1;
%             q2, dq2, ddq2]
%

% X0 = [-5, 0, 0; ...
%     15, 0, 0];    	% Waypoint 0 position, velocity, acceleration
% X1 = [270, 0, 0; ...
%     150, 0, 0];    % Waypoint 1 position, velocity, acceleration

X0 = [q0(1:4), q0(5:8), zeros(4,1)];  	% Waypoint 0 position, velocity, acceleration
X1 = [qgoal(1:4), qgoal(5:8), zeros(4,1)];    % Waypoint 1 position, velocity, acceleration

X = deg2rad(cat(3, X0, X1));