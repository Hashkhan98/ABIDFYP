function [q,qd,qdd] = TrajectoryGeneration(wpts,simspan,maxvel) 
    
[qtemp, qdtemp, qddtemp, tvec, pp] = trapveltraj(wpts, length(simspan), 'PeakVelocity' , maxvel);

q = zeros(4,size(qtemp,2));

for i = 1:size(q,2)
    [q(:,i),qd(:,i)] = IKM(qtemp(:,i),qdtemp(:,i));
end

% for i = 1:size(q,2)
%     qtemp(:,i) = IKM(q(:,i));
% end
% qd = gradient(q);

qdd = gradient(qd);

% figure(1)
% subplot(2,1,1)
% plot(tvec, qtemp)
% xlabel('t')
% ylabel('Positions')
% legend('X','Y')
% subplot(2,1,2)
% plot(tvec, qdtemp)
% xlabel('t')
% ylabel('Velocities')
% legend('X','Y')
% 
figure(2)
subplot(2,1,1)
plot(tvec, q)
xlabel('t')
ylabel('Joints')
legend('r1','r2','theta','z')
subplot(2,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')
% 
% figure(3)
% plot3(wpts(1,:),wpts(2,:),wpts(3,:))
end