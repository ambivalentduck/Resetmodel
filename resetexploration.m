clc
clear all

global l1 l2 w1 w2 x0

%%assume two link
l1=.5;
w1=1;
l2=.5;
w2=1;
x0=[0, -.5]; %Base of robot at half an arm length toward user from center of workspace
%Consequence: Workspace is a circle within the arm's semicircle of reach

ti=0;
tf=1;

syms x y t theta1 theta2 real;
theta=[theta1; theta2];
p=[x; y];

fk=fkin(theta);
ik=ikin(p);

p0=[-.2 -.2];
pf=[-.2 .1];

global coeff

coeff=calcminjerk(p0,pf,[0 0],[0 0],[0 0],[0 0],ti,tf);

%Command torques based on Jacobian, so build one

J2=[diff(ik(1),x,2),diff(ik(1),y,2);
    diff(ik(2),x,2),diff(ik(2),y,2)];

J=[diff(ik(1),x),diff(ik(1),y);
    diff(ik(2),x),diff(ik(2),y)];

J11=inline(vectorize(J(1,1)));
J21=inline(vectorize(J(2,1)));
J12=inline(vectorize(J(1,2)));
J22=inline(vectorize(J(2,2)));

J211=inline(vectorize(J2(1,1)));
J221=inline(vectorize(J2(2,1)));
J212=inline(vectorize(J2(1,2)));
J222=inline(vectorize(J2(2,2)));

global Jacobian

Jacobian=@(p,v,a) [J11(p(1),p(2)),J12(p(1),p(2));J21(p(1),p(2)),J22(p(1),p(2))]*a+[J211(p(1),p(2)),J212(p(1),p(2));J221(p(1),p(2)),J222(p(1),p(2))]*v.^2;


ini=ikin(p0);
[T,X]=ode45(@armdynamics,[0 1],[ini;0;0]);

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

