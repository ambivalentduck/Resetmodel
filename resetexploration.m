clc
clear all

global l1 w1 l2 w2 x0

%%assume two link
l1=.5;
w1=1;
l2=.5;
w2-1;
x0=[0, -.5]; %Base of robot at half an arm length toward user from center of workspace
%Consequence: Workspace is a circle within the arm's semicircle of reach

syms x y t theta1 theta2 real;
theta=[theta1; theta2];
p=[x; y];

fk=fkin(theta);
ik=ikin(p);

t=0:.01:1;
x=minjerk([-.2 -.2],[.2 .2],[0 0],[0 0],t);
T=length(t);
rxpast=zeros(2,T);

figure(1)
for k=1:T
    clf
    set(gca,'xlim',[-1 1])
    set(gca,'ylim',[-1 1])
    hold on
    plot(x(1,1:k),x(2,1:k),'b-')
    plot(x(1,k),x(2,k),'bx')
    plot(x0(1),x0(2),'rx')

    it=ikin(x(:,k));
    xt=fkin(it);
    plot(xt(1),xt(2),'r.')
    rxpast(:,k)=xt;
    x1=[x0(1)+l1*cos(it(1));x0(2)+l1*sin(it(1))];
    plot([x0(1),x1(1)],[x0(2),x1(2)],'r-')
    plot([x1(1),xt(1)],[x1(2),xt(2)],'r-')
    plot(rxpast(1,1:k),rxpast(2,1:k),'gx')
    F(k)=getframe;
end
movie(F)