function x=minjerk(xi,xf,v,duration,t)
%%goal use minimum jerk modeling
%Assume:
%-Shadmehr derived minimum jerk in 2d correctly
%-Time *resets* for each submovement
%-Start at [0,0] end at [0,1]
xi=[0 0];
xf=[0 1];
%-Time assumed to be 1s
a=1;
TF=duration;

K=length(T);
k=1;

xlist=[NaN*ones(3*K,1),NaN*ones(3*K,1)];
xlist(1,:)=xi;
xlistref=xlist;
vlist=[NaN*ones(3*K,1),NaN*ones(3*K,1)];
vlist(1,:)=[0 0];
alist=[NaN*ones(3*K,1),NaN*ones(3*K,1)];
alist(1,:)=[0 0];
alistref=alist;
listi=1;
t=0;
tmat=[1 t  t^2  t^3  t^4  t^5;
    1 TF TF^2 TF^3 TF^4 TF^5;
    0 1  2*t    3*t^2  4*t^3  5*t^4;
    0 1  2*TF   3*TF^2 4*TF^3 5*TF^4;
    0 0  2    6*t    12*t^2  20*t^3;
    0 0  2    6*TF   12*TF^2 20*TF^3];

xconstraints=[xlist(listi,1); xf(1); vlist(listi,1); 0; alist(listi,1); 0];
yconstraints=[xlist(listi,2); xf(2); vlist(listi,2); 0; alist(listi,2); 0];
xcoeff=tmat\xconstraints;
ycoeff=tmat\yconstraints;