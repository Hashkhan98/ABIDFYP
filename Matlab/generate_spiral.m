r= 0:0.01:1;
Theta = -pi:2*pi/100:pi;
z =repmat(0.4,1,101);
[x,y,z] = pol2cart(Theta,r,z)