function [q,qd,qdd,t,wpts] = TrajectoryGenerationNEW(maxvel,T) 

%%
wpts = csvread('LOGOxyztarg.csv');
wpts = wpts(:,1:3)'*0.0025;
wpts(1,:) = wpts(1,:) - 0.625;
wpts(3,:) = wpts(2,:)*0.1 + 0.5;
%%
[TG, t, info] = mstraj(wpts',[maxvel maxvel maxvel],[], [] ,T,0.1);

q = zeros(4,size(TG,1));

for i = 1:size(q,2)
    q(:,i) = IKM(TG(i,:),[],[]);
end

q0 = q(3,:)*10;
temp = unwrap(q0);
q(3,:) = temp/10;

qd = gradient(q)*100;
 
qdd = gradient(qd)*100;

% %%
% figure(1)
% 
% % subplot(3,1,1)
% plot(t(1:length(wpts)), wpts)
% xlabel('Time (s)')
% % xaxis([0 length(t)*T])
% ylabel('$\mathbf{Position(m)}$','interpreter','latex')
% legend('$X$','$Y$','$Z$','interpreter','latex','Location','northeastoutside')
% set(gca,'fontweight','bold','fontsize',18)
% subplot(3,1,2)
% v = [gradient(TG(:,1)),gradient(TG(:,2)),gradient(TG(:,3))];
% plot(t(1:length(wpts)),qd)
% xlabel('Time (s)')
% xaxis([0 length(t)*T])
% ylabel('$\mathbf{Velocity(\frac{m}{s}}$)','interpreter','latex')
% legend('$X$','$Y$','$Z$','interpreter','latex','Location','northeastoutside')
% set(gca,'fontweight','bold','fontsize',18)
% subplot(3,1,3)
% a = [gradient(v(:,1)),gradient(v(:,2)),gradient(v(:,3))];
% plot(t(1:length(wpts)), qdd)
% xlabel('t')
% ylabel('$\mathbf{Acceleration(\frac{m}{s^2}})$','interpreter','latex')
% legend('$X$','$Y$','$Z$','interpreter','latex','Location','northeastoutside')
% % legend(p([1:4]),'r1','r 2','z','','des r2','des theta','des z','theta')
% set(gcf,'color','w')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% sgtitle('\bf{Desired trajectory in Cartesian Coordinates}','fontsize',22,'interpreter','latex')
% % 
% %%
% figure(2)
% subplot(3,1,1)
% plot(t, q)
% xlabel('t')
% ylabel('$\mathbf{Position(m)}$','interpreter','latex')
% legend('$r_{1}$','$r_{2}$','$\theta$','$z$','interpreter','latex','Location','northeastoutside')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% xaxis([0 length(t)*T])
% subplot(3,1,2)
% plot(t, qd)
% ylabel('$\mathbf{Velocity(\frac{m}{s}}$)','interpreter','latex')
% legend('$r_{1}$','$r_{2}$','$\theta$','$z$','interpreter','latex','Location','northeastoutside')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% xaxis([0 length(t)*T])
% 
% subplot(3,1,3)
% plot(t, qdd)
% xlabel('t')
% ylabel('Acceleration')
% ylabel('$\mathbf{Acceleration(\frac{m}{s^2}})$','interpreter','latex')
% legend('$r_{1}$','$r_{2}$','$\theta$','$z$','interpreter','latex','Location','northeastoutside')
% % legend(p([1:4]),'r1','r 2','z','','des r2','des theta','des z','theta')
% set(gcf,'color','w')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% xaxis([0 length(t)*T])
% sgtitle('\bf{Desired trajectory in Joint Coordinates}','fontsize',22,'interpreter','latex')
% %%
% figure(3)
% [X,Y,Z] = pol2cart(q(3,:),q(1,:),q(4,:));
% plot3(X(:),Y(:),Z(:),'b.-','MarkerSize',10)
% title('University of Newcastle Logo as waypoints')
% xlabel('X (m)')
% ylabel('Y (m)')
% set(gcf,'color','w')
% set(gca,'fontweight','bold','fontsize',22)
% view(0,90)
% %%
figure(4)
plot3(wpts(1,:),wpts(2,:),wpts(3,:),'r.-','MarkerSize',20)
axis([-1 1 -1 1 0 1]);    
view(0,90)
title('University of Newcastle Logo as Featurepoints')
xlabel('X (m)')
ylabel('Y (m)')
set(gcf,'color','w')
set(gca,'fontweight','bold','fontsize',22)
end