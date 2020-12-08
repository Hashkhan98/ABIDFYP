function out = trajectoryOutputHelper(t)

global param

X = trajectoryEval(param.traj, t);

out = [X(1,:).'; X(2,:).'];     % [q0; dq0; ddq0; q1; dq1; ddq1];