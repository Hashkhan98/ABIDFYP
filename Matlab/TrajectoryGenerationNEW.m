function [q,qd,qdd,t] = TrajectoryGenerationNEW(wpts,maxvel) 

%%
    
[TG, t, info] = mstraj(wpts',[maxvel maxvel maxvel],[], [] ,0.01,0.1);

q = zeros(4,size(TG,1));
 
for i = 1:size(q,2)
    q(:,i) = IKM(TG(i,:),[],[]);
end
qd = gradient(q)*100;
 
qdd = gradient(qd)*100;

%%
% figure(1)
% subplot(3,1,1)
% plot(t, TG)
% xlabel('t')
% ylabel('Positions')
% legend('X','Y','Z')
% subplot(3,1,2)
% v = [gradient(TG(:,1)),gradient(TG(:,2)),gradient(TG(:,3))];
% plot(t,v)
% xlabel('t')
% ylabel('Velocities')
% legend('X','Y','Z')
% subplot(3,1,3)
% a = [gradient(v(:,1)),gradient(v(:,2)),gradient(v(:,3))];
% plot(t, a)
% xlabel('t')
% ylabel('Acceleration')
% legend('X','Y')
% % 
% 
% figure(2)
% subplot(3,1,1)
% plot(t, q)
% xlabel('t')
% ylabel('Joints')
% legend('r1','r2','theta','z')
% subplot(3,1,2)
% plot(t, qd)
% xlabel('t')
% ylabel('Velocities')
% legend('X','Y')
% subplot(3,1,3)
% plot(t, qdd)
% xlabel('t')
% ylabel('Acceleration')
% legend('X','Y')
% %%
% figure(3)
% [X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
% plot3(X(:),Y(:),Z(:),'b-')
% hold on
% % [X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
% plot3(wpts(1,:),wpts(2,:),wpts(3,:),'r.-')
% axis([-1 1 -1 1 0 1]);    
% view(0,90)
% hold off
% % figure(4)
% % plot3(wpts(1,:),wpts(2,:),wpts(3,:))
end