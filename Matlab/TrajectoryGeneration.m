function [qtemp,qd,qdd] = TrajectoryGeneration() 

wpts = [+0.5 +0.5 +0.5 +0.5;
        -0.5 +0.5 +0.5 -0.5;
        0.3 0.4 0.3 0.4 ];

[q, qd, qdd, tvec, pp] = trapveltraj(wpts, 501);

qtemp = zeros(4,size(q,2));

for i = 1:size(q,2)
    qtemp(:,i) = IKM(q(:,i));
end

for i = 1:size(q,2)
    qtemp(:,i) = IKM(q(:,i));
end
qd = zeros(4,size(q,2));

qdd = zeros(4,size(q,2));
% figure(1)
% subplot(2,1,1)
% plot(tvec, q)
% xlabel('t')
% ylabel('Positions')
% legend('X','Y')
% subplot(2,1,2)
% plot(tvec, qd)
% xlabel('t')
% ylabel('Velocities')
% legend('X','Y')
% 
% figure(2)
% subplot(2,1,1)
% plot(tvec, qtemp)
% xlabel('t')
% ylabel('Joints')
% legend('r1','r2','theta','z')
% subplot(2,1,2)
% plot(tvec, qd)
% xlabel('t')
% ylabel('Velocities')
% legend('X','Y')
end