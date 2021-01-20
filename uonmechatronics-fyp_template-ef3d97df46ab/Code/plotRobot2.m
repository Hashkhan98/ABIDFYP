%% Plotting
figure(5)
fig(1) = plot3([0,0],[0,0], [0, 1], 'b-','LineWidth',6,...
    'MarkerSize',4,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);
hold on
axis([-1 1 -1 1 0 1]);    
view(0,90)
title('Robot tracking desired trajectory')
xlabel('x','Fontsize',20);
ylabel('y','Fontsize',20);
zlabel('z','Fontsize',20);
set(gcf,'color','w');
fig(2) = plot3([rB_mc1(1), rB_beam_m1(1)],[rB_mc1(2), rB_beam_m1(2)], ...
    [rB_mc1(3), rB_beam_m1(3)],'b-','LineWidth',6,'MarkerSize',4,...
    'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);
fig(3) = plot3([rB_mc1(1), rB_beam_m2(1)],[rB_mc1(2), rB_beam_m2(2)],...
    [rB_mc1(3), rB_beam_m2(3)],'b-','LineWidth',6,'MarkerSize',4,...
    'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);
fig(4) = plot3([rB_m1(1)],[rB_m1(2)], [rB_m1(3)], 'rs','LineWidth',6,...
    'MarkerSize',4*2,'MarkerFaceColor','k','MarkerEdgeColor','k');
fig(5) = plot3([rB_m2(1)],[rB_m2(2)], [rB_m2(3)], 'rs','LineWidth',6,...
    'MarkerSize',4*4,'MarkerFaceColor','k','MarkerEdgeColor','k');
fig(6) = plot3(0,0, rB_m2(3), 'ro','LineWidth',6,'MarkerSize',4*3,...
    'MarkerFaceColor','k','MarkerEdgeColor','k');
fig(10) = plot3([rB_m1(1)],[rB_m1(2)], [rB_m1(3)], 'rs','LineWidth',2,...
    'MarkerSize',2,'MarkerFaceColor','r','MarkerEdgeColor','r');
[x1 , y1] = meshgrid(-1:0.1:1); % Generate x and y data
z1 = zeros(size(x1, 1)); % Generate z data
surf(x1, y1, z1) % Plot the surface
drawnow
if flag
    set(gcf,'color','w')
    set(gca,'fontweight','bold','fontsize',22)
    legend(fig([1,4,5,10]),{'Robot Frame','Mass 1','Mass 2',...
        'Trajectory'},'interpreter','latex','Location','northeastoutside')
    xlabel('x (m)') 
    ylabel('y (m)')
    zlabel('z (m)')
else
delete([fig(1),fig(2),fig(3),fig(4),fig(5),fig(6)])
end
end