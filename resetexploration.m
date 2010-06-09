clc
clear all

global l1 l2 z x0 lasterror pf coeff reset Jacobian Jacobian2 fJacobian fJacobian2

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

p0=[.1 .2];
pf=[.1 -.2];

coeff(1).vals=calcminjerk(p0,pf,[0 0],[0 0],[0 0],[0 0],ti,tf);
coeff(1).expiration=tf;

%Command torques based on Jacobian, so build one
[Jacobian, Jacobian2, fJacobian, fJacobian2]=makeJacobians;

for reset=0 %:2
    ini=ikin(p0);
    [T,X]=ode45(@armdynamics,[0 3],[ini;0;0]);

    LT=length(T);
    command=zeros(2,LT);
    armpos=zeros(2,LT);
    joint1=zeros(2,LT);

    figure(reset+1)
    for k=1:length(T)
        clf
        hold on

        [armpos(:,k),joint1(:,k)]=fkin(X(k,1:2));
        plot(x0(1),x0(2),'bx')
        plot([x0(1), joint1(1,k)],[x0(2), joint1(2,k)],'b-')
        plot([joint1(1,k),armpos(1,k)],[joint1(2,k),armpos(2,k)],'b-')
        plot(joint1(1,1:k),joint1(2,1:k),'g.')
        plot(armpos(1,1:k),armpos(2,1:k),'g.')

        command(:,k)=minjerk(coeff{1},T(k));
        plot(command(1,1:k),command(2,1:k),'r-')
        plot(command(1,k),command(2,k),'rx')


        set(gca,'xlim',[-1,1]);
        set(gca,'ylim',[-1,1]);
        F(k)=getframe; %#ok<AGROW>
    end
    movie(F)
end
