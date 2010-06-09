function dx=dynamics(t,x)

global l1 l2 I1 I2 coeff Jacobian Jacobian2

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
torque_outside=[(-l1*sin(x(1))-l2*sin(12))*F(1)+(l1*cos(x(1))+l2*c12)*F(2);
        -l2*s12*F(1)+l2*c12*F(2)];

dx=[x(3);
    x(4);
    alpha(1)+alpha_feedback(1)+torque_outside(1)/I1;
    alpha(2)+alpha_feedback(2)+torque_outside(2)/I2];

end

function out=getforces(t)
out=[500;0]*((t<.55)&&(t>.5));
end