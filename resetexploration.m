clc
clear all

global l1 l2 m1 m2 I1 I2 z x0 lastreset pf coeff reset Jacobian Jacobian2 fJacobian fJacobian2

%%assume two link
l1=1;
l2=1;
m1=1;
m2=1;
%model as solid rod
I1=(m1*l1^2)/3;
I2=(m2*l2^2)/3;

z=[m2*l1*(l2/2);
    I2+m2*(l2/2)^2;
    I1+I2+(m1*l1^2+m2*l2^2)/4+m2*l1^2;
    m2*l1*l2];

x0=[0, -.5]; %Base of robot at half an arm length toward user from center of workspace
%Consequence: Workspace is a circle within the arm's semicircle of reach

ti=0;
tf=1;

p0=[.2 0];
pf=[-.2 0];

coeff(1).vals=calcminjerk(p0,pf,[0 0],[0 0],[0 0],[0 0],ti,tf);
coeff(1).expiration=tf;
coeff(1).stale=inf;

%Command torques based on Jacobian, so build one
[Jacobian, Jacobian2, fJacobian, fJacobian2]=makeJacobians;

titles={'Type 0 Reset: None','Type 1 Reset: feedback control sees updated trajectory','Type 2 Reset: Feedback Updated and corrective submovement initiated'};

for res=0:2
    C=1;
    lastreset=-inf;
    coeff=cell(0);
    coeff(1).vals=calcminjerk(p0,pf,[0 0],[0 0],[0 0],[0 0],ti,tf);
    coeff(1).expiration=tf;
    coeff(1).stale=inf;
    
    reset=res;
    ini=ikin(p0);
    %[T,X]=ode45(@armdynamics,[0 3],[ini;0;0]);
    [T,X]=eulerode(@armdynamics,[0 3],[ini;0;0]);

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
        %plot(joint1(1,1:k),joint1(2,1:k),'g.')
        plot(armpos(1,1:k),armpos(2,1:k),'g.')

        while T(k)>coeff(C).stale
            C=C+1;
        end
        if T(k)>coeff(C).expiration
            command(:,k)=minjerk(coeff(C).vals,coeff(C).expiration);
        else
            command(:,k)=minjerk(coeff(C).vals,T(k));
        end
        plot(command(1,1:k),command(2,1:k),'r-')
        plot(command(1,k),command(2,k),'rx')


        set(gca,'xlim',[-1,1]);
        set(gca,'ylim',[-1,1]);
        F(k)=getframe; %#ok<AGROW>
    end
    title(titles{reset+1})
    if (C-1)==1
        xlabel('There was 1 reset.')
    else
        xlabel(['There were ',num2str(C-1),' resets.'])
    end
    print('-djpeg',['reset',num2str(reset),'.jpg'])
    %movie(F)
end
