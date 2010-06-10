clc
clear all

coeff1=calcminjerk([0 -1],[0 1],[0 0],[0 0],[0 0],[0 0],0,1);
[p,v,a]=minjerk(coeff1,.5);
coeff2=calcminjerk(p,[0,1],v+[1;0],[0 0],a,[0 0],.5,1);

figure(5)
clf
hold on
t=0:.01:1;
x1=minjerk(coeff1,t);
plot(x1(1,:),x1(2,:))
t=.5:.01:1;
x2=minjerk(coeff2,t);
plot(x2(1,:),x2(2,:),'rx')


