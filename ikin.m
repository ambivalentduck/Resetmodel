function theta=ikin(x)

global l1 l2 x0

if(isa(x,'sym'))
    gamma=acos(((x(1)-x0(1))^2+(x(2)-x0(2))^2-l1^2-l2^2)/(-2*l1*l2));
    theta2=pi-gamma;
    theta1=atan(x(2)-x0(2),x(1)-x0(1))-theta2/2;
else
    gamma=abs(acos(((x(1)-x0(1))^2+(x(2)-x0(2))^2-l1^2-l2^2)/(-2*l1*l2)));
    theta2=pi-gamma;
    theta1=atan2(x(2)-x0(2),x(1)-x0(1))-theta2/2;
end

theta=[theta1; theta2];
