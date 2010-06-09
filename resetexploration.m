clc
clear all

global l1 l2 w1 w2 z x0 lasterror pf coeff Jacobian Jacobian2 fJacobian fJacobian2

lasterror=inf;

%%assume two link
l1=.5;
l2=.5;
w1=1;
w2=1;
%model as solid rod
I1=(w1*l1^2)/3;
I2=(w2*l2^2)/3;

z=[w2*l1*(l2/2);
   I2+w2*(l2/2)^2;
   I1+I2+(w1*l1^2+w2*l2^2)/4+w2*l1^2;
   w2*l1*l2];

x0=[0, -.5]; %Base of robot at half an arm length toward user from center of workspace
%Consequence: Workspace is a circle within the arm's semicircle of reach

ti=0;
tf=1;

syms x y t theta1 theta2 real;
theta=[theta1; theta2];
p=[x; y];

fk=fkin(theta);
ik=ikin(p);

p0=[.1 .2];
pf=[.1 -.2];

coeff{1}=calcminjerk(p0,pf,[0 0],[0 0],[0 0],[0 0],ti,tf);

%Command torques based on Jacobian, so build one

J11=inline(vectorize(diff(ik(1),x)));
J21=inline(vectorize(diff(ik(2),x)));
J12=inline(vectorize(diff(ik(1),y)));
J22=inline(vectorize(diff(ik(2),y)));

Jt1dx2=inline(vectorize(diff(ik(1),x,2)));
Jt1dy2=inline(vectorize(diff(ik(1),y,2)));
Jt2dx2=inline(vectorize(diff(ik(2),x,2)));
Jt2dy2=inline(vectorize(diff(ik(2),y,2)));
Jt1dxdy=inline(vectorize(diff(diff(ik(1),x),y)));
Jt2dxdy=inline(vectorize(diff(diff(ik(2),x),y)));

Jacobian=@(p,v,a) [J11(p(1),p(2)),J12(p(1),p(2));J21(p(1),p(2)),J22(p(1),p(2))]*v;
Jacobian2=@(p,v,a) [J11(p(1),p(2)),J12(p(1),p(2));J21(p(1),p(2)),J22(p(1),p(2))]*a+[Jt1dx2(p(1),p(2))*v(1)+Jt1dxdy(p(1),p(2))*v(2),Jt1dy2(p(1),p(2))*v(2)+Jt1dxdy(p(1),p(2))*v(2);Jt2dx2(p(1),p(2))*v(1)+Jt2dxdy(p(1),p(2))*v(2),Jt2dy2(p(1),p(2))*v(2)+Jt2dxdy(p(1),p(2))*v(2)]*v;

fJ11=inline(vectorize(diff(fk(1),x)));
fJ21=inline(vectorize(diff(fk(2),x)));
fJ12=inline(vectorize(diff(fk(1),y)));
fJ22=inline(vectorize(diff(fk(2),y)));

fJt1dx2=inline(vectorize(diff(fk(1),x,2)));
fJt1dy2=inline(vectorize(diff(fk(1),y,2)));
fJt2dx2=inline(vectorize(diff(fk(2),x,2)));
fJt2dy2=inline(vectorize(diff(fk(2),y,2)));
fJt1dxdy=inline(vectorize(diff(diff(fk(1),x),y)));
fJt2dxdy=inline(vectorize(diff(diff(fk(2),x),y)));

Jacobian=@(p,v,a) [J11(p(1),p(2)),J12(p(1),p(2));J21(p(1),p(2)),J22(p(1),p(2))]*v;
Jacobian2=@(p,v,a) [J11(p(1),p(2)),J12(p(1),p(2));J21(p(1),p(2)),J22(p(1),p(2))]*a+[Jt1dx2(p(1),p(2))*v(1)+Jt1dxdy(p(1),p(2))*v(2),Jt1dy2(p(1),p(2))*v(2)+Jt1dxdy(p(1),p(2))*v(2);Jt2dx2(p(1),p(2))*v(1)+Jt2dxdy(p(1),p(2))*v(2),Jt2dy2(p(1),p(2))*v(2)+Jt2dxdy(p(1),p(2))*v(2)]*v;


ini=ikin(p0);
[T,X]=ode45(@armdynamics,[0 2],[ini;0;0]);

LT=length(T);
command=zeros(2,LT);
armpos=zeros(2,LT);
joint1=zeros(2,LT);

figure(1)
for k=1:length(T)
    clf
    hold on
    
    [armpos(:,k),joint1(:,k)]=fkin(X(k,1:2));
    plot(x0(1),x0(2),'bx')
    plot([x0(1), joint1(1,k)],[x0(2), joint1(2,k)],'b-')
    plot([joint1(1,k),armpos(1,k)],[joint1(2,k),armpos(2,k)],'b-')
    plot(joint1(1,1:k),joint1(2,1:k),'g.')
    plot(armpos(1,1:k),armpos(2,1:k),'g.')
    
    command(:,k)=minjerk(coeff,T(k));
    plot(command(1,1:k),command(2,1:k),'r-')
    plot(command(1,k),command(2,k),'rx')


    set(gca,'xlim',[-1,1]);
    set(gca,'ylim',[-1,1]);
    F(k)=getframe;
end
movie(F)

