function dx=armdynamics(t,x)

global l1 l2 z coeff pf Jacobian Jacobian2 fJacobian fJacobian2 reset lasterror

%x(1-2) are joint angle
%x(3-4) are velocity

alpha=0;

if(reset>=1) %Type >=1 reset changes feedback controller.
    K=length(coeff);
else
    K=1;
end

for k=1:K
    if(coeff(k).expiration<=t)
        [p,v,a]=minjerk(coeff(k).vals,t);
    else
        [p,v,a]=minjerk(coeff(k).vals,coeff(k).expiration); %Final values stand forever
    end
    % J is d2_theta/dp2, a is d2p/dt2, so J*a is d2_theta/dt2 via chain rule
    if((reset>=2)||(k==1)) %Ie. Type >=2 reset includes superposition well-evidenced in literature
        alpha=alpha+Jacobian2(p,v,a)*(coeff(k).expiration<=t);
    end
end

%Add feedback
omega_desired=Jacobian(p,v,a);
theta_desired=ikin(p);
kd=0; %-10;
kp=0; %-50;
alpha_feedback=kd*(x(3:4)-omega_desired)+kp*(x(1:2)-theta_desired);

%Add torque due to outside forces
F=getforces(t);
s12=sin(x(1)+x(2));
c12=cos(x(1)+x(2));
c2=cos(x(2));

%Copy the mechanics from Jim's code...
I=[z(3)+z(4)*c2, z(2)+z(1)*c2;
    z(2)+z(1)*c2, z(2)];

inter=z(1)*sin(x(2));
g=[-inter*(x(4)^2+2*x(3)*x(4));
    inter*x(3)^2];

torque_outside=[(-l1*sin(x(1))-l2*sin(12))*F(1)+(l1*cos(x(1))+l2*c12)*F(2);
    -l2*s12*F(1)+l2*c12*F(2)];

dx=[x(3);
    x(4);
    alpha+alpha_feedback+inv(I)*(torque_outside-g)];
%I assume a *perfect* inverse model to produce the right torque and just specify angular
%acceleration instead.  Why bother adding numerical noise by multiplying alpha terms by I*I^-1 ?

p_real=fkin(x(1:2));
v_real=fJacobian(x(1:2),x(3:4),[0 0]);
a_real=fJacobian2(x(1:2),x(3:4),dx(3:4));
e=sum((p_real-p).^2); %Note that this p is always from the most "updated" ff trajectory and only it
if( (e>.1)&&(e>lasterror)&&(reset>=1) ) %Reset Condition + getting worse + reset desired
    coeff{end+1}=calcminjerk(p_real,pf,v_real,[0 0],a_real,[0 0],t,t+1);
end

end


function out=getforces(t)
out=[500;0]*((t<.55)&&(t>.5));
end