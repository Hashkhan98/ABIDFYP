function [q,qd,qdd] = TrajectoryGeneration(wpts,simspan,maxvel) 
    
[qtemp, qdtemp, qddtemp, tvec, pp] = trapveltraj(wpts, length(simspan));

q = zeros(4,size(qtemp,2));
qd = q;
qdd = qd;

for i = 1:size(q,2)
    q(:,i) = IKM(qtemp(:,i),qdtemp(:,i),qddtemp(:,i));
end

qd = gradient(q)*100;

qdd = gradient(qd)*100;

figure(1)
subplot(3,1,1)
plot(tvec, qtemp)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(3,1,2)
plot(tvec, qdtemp)
xlabel('t')
ylabel('Velocities')
legend('X','Y')
subplot(3,1,3)
plot(tvec, qddtemp)
xlabel('t')
ylabel('Acceleration')
legend('X','Y')
% 

figure(2)
subplot(3,1,1)
plot(tvec, q)
xlabel('t')
ylabel('Joints')
legend('r1','r2','theta','z')
subplot(3,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')
subplot(3,1,3)
plot(tvec, qdd)
xlabel('t')
ylabel('Acceleration')
legend('X','Y')

figure(3)
[X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
plot3(X(:),Y(:),Z(:),'b--')
hold on
% [X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
plot3(qtemp(1,:),qtemp(2,:),qtemp(3,:),'r--')
axis([-1 1 -1 1 0 1]);    
view(0,90)
hold off
% figure(3)
% plot3(wpts(1,:),wpts(2,:),wpts(3,:))
end