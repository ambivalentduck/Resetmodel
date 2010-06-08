function dx=dynamics(t,x)

global coeff Jacobian

[p,v,a]=minjerk(coeff,t);

% J is d2_theta/dp2, a is d2p/dt2, so J*a is d2_theta/dt2 via chain rule
alpha=Jacobian(p,v,a);

%x(1-2) are joint angle
%x(3-4) are velocity

dx=[x(3);
    x(4);
    alpha(1);
    alpha(2)];
    
    

