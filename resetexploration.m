clc
clear all

global l1 w1 l2 w2 x0

%%assume two link
l1=.5;
w1=1;
l2=.5;
w2=1;
x0=[0, -.5]; %Base of robot at half an arm length toward user from center of workspace
%Consequence: Workspace is a circle within the arm's semicircle of reach

syms x y t theta1 theta2 real;
theta=[theta1; theta2];
p=[x; y];

fk=fkin(theta);
ik=ikin(p);

t=0:.01:1;
[xd,vd,ad]=minjerk([-.2 -.2],[.2 .2],[0 0],[0 0],t);
T=length(t);

%Command torques based on Jacobian, so build one

J=[diff(ik(1),x,2),diff(ik(1),y,2);
    diff(ik(2),x,2),diff(ik(2),y,2)];

J11=inline(vectorize(J(1,1)));
J21=inline(vectorize(J(2,1)));
J12=inline(vectorize(J(1,2)));
J22=inline(vectorize(J(2,2)));

Jacobian=@(x, y) [J11(x,y),J12(x,y);J21(x,y),J22(x,y)];

% J is d2_ik/dp2, so J*a is d2_ik/dt2 via chain rule

global alphadesired

alphadesired=zeros(2,T);
for k=1:T
    alphadesired(:,k)=Jacobian(xd(1,k),xd(2,k))*ad(:,k);
end