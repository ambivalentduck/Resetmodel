function x=fKin(theta)

global l1 l2 x0
x=[x0(1)+l1*cos(theta(1))+l2*cos(theta(1)+theta(2));
   x0(2)+l1*sin(theta(1))+l2*sin(theta(1)+theta(2))];
