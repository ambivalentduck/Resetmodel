function [x,v,a]=minjerk(coeff,tlist)

x=zeros(2,length(tlist));
v=zeros(2,length(tlist));
a=zeros(2,length(tlist));
for k=1:length(tlist)
    t=tlist(k);
    tmat=[1 t  t^2  t^3  t^4  t^5;
    1 TF TF^2 TF^3 TF^4 TF^5;
    0 1  2*t    3*t^2  4*t^3  5*t^4;
    0 1  2*TF   3*TF^2 4*TF^3 5*TF^4;
    0 0  2    6*t    12*t^2  20*t^3;
    0 0  2    6*TF   12*TF^2 20*TF^3];
    x(:,k)=[dot(xcoeff,tmat(1,:));dot(ycoeff,tmat(1,:))];
    v(:,k)=[dot(xcoeff,tmat(3,:));dot(ycoeff,tmat(3,:))];
    a(:,k)=[dot(xcoeff,tmat(5,:));dot(ycoeff,tmat(5,:))];
end