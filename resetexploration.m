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



