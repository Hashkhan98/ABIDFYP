function [fig] = drawFrame(A0ee,fig)

%Rotate coordinate frame T00 by A01
x_line = A0ee*[A0ee(1:3,1:3),[0.05;0;0];[0,0,0,1]];
y_line = A0ee*[A0ee(1:3,1:3),[0;0.05;0];[0,0,0,1]];
z_line = A0ee*[A0ee(1:3,1:3),[0;0;0.05];[0,0,0,1]];


fig(4) = plot3([A0ee(1,4);x_line(1,4)],[A0ee(2,4);x_line(2,4)],[A0ee(3,4);x_line(3,4)],'r-','LineWidth',5);
hold on
plot3([A0ee(1,4);y_line(1,4)],[A0ee(2,4);y_line(2,4)],[A0ee(3,4);y_line(3,4)],'g-','LineWidth',5)
plot3([A0ee(1,4);z_line(1,4)],[A0ee(2,4);z_line(2,4)],[A0ee(3,4);z_line(3,4)],'y-','LineWidth',5)
% legend('Robot','Gripper','x','y','z')


