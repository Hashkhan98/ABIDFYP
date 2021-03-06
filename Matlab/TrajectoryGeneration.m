function [q,qd,qdd] = TrajectoryGeneration(wpts,simspan,maxvel) 


%% Generate trajectory

Dist = zeros(size(wpts,2)-1,1);
Time = Dist;
qtemp = [];
qdtemp = qtemp;
qddtemp = qdtemp;
tvec = [];

for i = 1:size(wpts,2) - 1
    %%Find Absolute Distance using D = sqrt((x2 - x1)^2 + (z2 - z1)^2 + (z2 - z1)^2) 
    Dist(i) = sqrt((wpts(3*i-2) - wpts(3*i+1))^2 + (wpts(3*i-1) - wpts(3*i+2))^2 + (wpts(3*i) - wpts(3*i+3))^2);
    %%Time = Distance / Speed
    Time(i) = Dist(i) / maxvel;
    [qt, qdt, qddt, tv, ~] = trapveltraj([wpts(:,i),wpts(:,i+1)], int16(round(Time(i)*100,3)),'EndTime',Time(i));
    qtemp = [qtemp,qt];
    qdtemp = [qdtemp,qdt];
    qddtemp = [qddtemp,qddt];
    tvec = [tvec,tv];
end


%%
% [qtemp, qdtemp, qddtemp, tvec, ~] = trapveltraj(wpts, length(simspan),'EndTime',7);
% [qtemp, qdtemp, qddtemp, tvec, pp] = trapveltraj(wpts, length(simspan),'PeakVelocity',[0.25;0.25;0.25]);

q = zeros(4,size(qtemp,2));
qd = q;
qdd = qd;

for i = 1:size(q,2)
    q(:,i) = IKM(qtemp(:,i),qdtemp(:,i),qddtemp(:,i));
end

q0 = q(3,:)*10;
temp = unwrap(q0);
q(3,:) = temp/10;

%% impose Minimum Resolution 

minResolution = 0.001;
D = [];
for i = 1:size(q,2) - 1
    %%Find Absolute Distance using D = sqrt((x2 - x1)^2 + (z2 - z1)^2 + (z2 - z1)^2) 
    D(i) = sqrt((q(4*i-3) - q(4*i+1))^2 + (q(4*i-2) - q(4*i+2))^2 + (q(4*i) - q(4*i+4))^2);
end
newq = q(:,(find(D > minResolution)));

%%
qd = gradient(q)*100;

qdd = gradient(qd)*100;


%%
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
t = tvec(1:length(q));

figure(2)
subplot(3,1,1)
plot(t, q)
xlabel('t')
ylabel('Joints')
legend('r1','r2','theta','z')
subplot(3,1,2)
plot(t, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')
subplot(3,1,3)
plot(t, qdd)
xlabel('t')
ylabel('Acceleration')
legend('X','Y')

figure(3)
[X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
plot3(X(:),Y(:),Z(:),'b.-')
hold on
% [X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
plot3(qtemp(1,:),qtemp(2,:),qtemp(3,:),'r--')
axis([-1 1 -1 1 0 1]);    
view(0,90)
hold off
figure(4)
plot3(wpts(1,:),wpts(2,:),wpts(3,:))
end