function dx=dynamics(t,x)

global l1 l2 w1 w2 I1 I2 z coeff pf Jacobian Jacobian2 fJacobian fJacobian2 reset lasterror

[p,v,a]=minjerk(coeff,t);

% J is d2_theta/dp2, a is d2p/dt2, so J*a is d2_theta/dt2 via chain rule
alpha=Jacobian2(p,v,a);

%x(1-2) are joint angle
%x(3-4) are velocity

%Add feedback
omega_desired=Jacobian(p,v,a);
theta_desired=ikin(p);
kd=-10;
kp=-50;
alpha_feedback=kd*(x(3:4)-omega_desired)+kp*(x(1:2)-theta_desired);

%Add torque due to outside forces
F=getforces(t);
s12=sin(x(1)+x(2));
c12=cos(x(1)+x(2));
c2=cos(x(2));

I=[z(3)+z(4)*c2, z(2)+z(1)*c2;
   z(2)+z(1)*c2, z2];

inter=z(1)*sin(x(2));
g=[-inter*(x(4)^2+2*x(3)*x(4));
    inter*s(3)^2];

torque_outside=[(-l1*sin(x(1))-l2*sin(12))*F(1)+(l1*cos(x(1))+l2*c12)*F(2);
    -l2*s12*F(1)+l2*c12*F(2)];

dx=[x(3);
    x(4);
    alpha+alpha_feedback+inv(I)*(torque_outside-g)];

%I assume a *perfect* inverse model to produce the right torque and just specify angular
%acceleration instead.  Why bother adding numerical noise by multiplying by I*I^-1 ?

p_real=fkin(x(1:2));
v_real=fJacobian(x);
a_real=fJacobian2([x;dx(3:4)]);
e=sum((p_real-p).^2);
if( (e>.1)&&(e>lasterror) ) %Reset Condition + getting worse
    coeff{end+1}=calcminjerk(p_real,pf,v_real,[0 0],a_real,[0 0],t,t+1);
end

end


function out=getforces(t)
out=[500;0]*((t<.55)&&(t>.5));
end