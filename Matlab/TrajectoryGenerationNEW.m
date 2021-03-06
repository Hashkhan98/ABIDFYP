function [q,qd,qdd,t,wpts] = TrajectoryGenerationNEW(maxvel,T) 

%%
wpts = csvread('LOGOxyztarg.csv');
wpts = wpts(:,1:3)'*0.0025;
wpts(1,:) = wpts(1,:) - 0.625;
p = sin([1:0.3:length(wpts)]);
wpts(3,:) = p(1:length(wpts))*0.05 + 0.5;
%%
[TG, t, ~] = mstraj(wpts',[maxvel maxvel maxvel],[], [] ,T,0.1);

q = zeros(4,size(TG,1));

p = 0.05*sin(1:0.01:length(TG));
TG(:,3) = p(1:length(TG)) + 0.5;

for i = 1:size(q,2)
    q(:,i) = IKM(TG(i,:),[],[]);
end

q0 = q(3,:)*10;
temp = unwrap(q0);
q(3,:) = temp/10;

qd = gradient(q)*100;
 
qdd = gradient(qd)*100;

%  %
% figure(1)
% 
% subplot(3,1,1)
% plot(t(1:length(TG(:,1))), TG)
% xaxis([0 length(t)*T])
% xlabel('Time (s)')
% % xaxis([0 length(t)*T])
% ylabel('$\mathbf{Position(m)}$','interpreter','latex')
% legend('$X$','$Y$','$Z$','interpreter','latex','Location','northeastoutside')
% set(gca,'fontweight','bold','fontsize',18)
% subplot(3,1,2)
% v = [gradient(TG(:,1)),gradient(TG(:,2)),gradient(TG(:,3))];
% plot(t(1:length(v)),v)
% xlabel('Time (s)')
% xaxis([0 length(t)*T])
% ylabel('$\mathbf{Velocity(\frac{m}{s}}$)','interpreter','latex')
% legend('$X$','$Y$','$Z$','interpreter','latex','Location','northeastoutside')
% set(gca,'fontweight','bold','fontsize',18)
% subplot(3,1,3)
% a = [gradient(v(:,1)),gradient(v(:,2)),gradient(v(:,3))];
% plot(t(1:length(a)), a)
% xaxis([0 length(t)*T])
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
% plot(t, q([1,2,4],:))
% xlabel('t')
% ylabel('$\mathbf{Position(m)}$','interpreter','latex')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% xaxis([0 length(t)*T])
% hold on
% 
% yyaxis right
% ax = gca;
% ax.YColor = [20 155 20]/256;
% plot(t, q(3,:),'color',[20 155 20]/256)
% ylabel('$\mathbf{Position(rad)}$','interpreter','latex')
% legend('$r_{1}$','$r_{2}$','$z$','$\theta$','interpreter','latex','Location','northeastoutside')
% hold off
% 
% subplot(3,1,2)
% 
% plot(t, qd([1,2,4],:))
% ylabel('$\mathbf{Velocity(\frac{m}{s}}$)','interpreter','latex')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% 
% hold on
% 
% yyaxis right
% ax = gca;
% ax.YColor = [20 155 20]/256;
% plot(t, qd(3,:),'color',[20 155 20]/256)
% ylabel('$\mathbf{Velocity(\frac{rad}{s})}$','interpreter','latex')
% legend('$\dot{r_{1}}$','$\dot{r_{2}}$','$\dot{z}$','$\dot{\theta}$','interpreter','latex','Location','northeastoutside')
% 
% hold off
% 
% subplot(3,1,3)
% plot(t, qdd([1,2,4],:))
% xlabel('t')
% ylabel('Acceleration')
% ylabel('$\mathbf{Acceleration(\frac{m}{s^2}})$','interpreter','latex')
% % legend(p([1:4]),'r1','r 2','z','','des r2','des theta','des z','theta')
% set(gcf,'color','w')
% set(gca,'fontweight','bold','fontsize',18)
% xlabel('Time (s)')
% 
% hold on
% 
% yyaxis right
% ax = gca;
% ax.YColor = [20 155 20]/256;
% plot(t, qdd(3,:),'color',[20 155 20]/256)
% ylabel('$\mathbf{Acceleration(\frac{rad}{s^2})}$','interpreter','latex')
% legend('$\ddot{r_{1}}$','$\ddot{r_{2}}$','$\ddot{z}$','$\ddot{\theta}$','interpreter','latex','Location','northeastoutside')
% 
% hold off
% 
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
% figure(4)
% plot3(wpts(1,:),wpts(2,:),wpts(3,:),'r.-','MarkerSize',20)
% axis([-1 1 -1 1 0 1]);    
% view(0,90)
% title('University of Newcastle Logo as Featurepoints')
% xlabel('X (m)')
% ylabel('Y (m)')
% set(gcf,'color','w')
% set(gca,'fontweight','bold','fontsize',22)
end