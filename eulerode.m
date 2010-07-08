function [T,X,fhandle,fapplied]=eulerode(fun, time, ini)

T=time(1):0.01:time(end);
X=zeros(length(ini),length(T));
fhandle=zeros(2,length(T));
fapplied=zeros(2,length(T));
X(:,1)=ini;

for k=2:length(T)
    [dX,handle,applied]=fun(T(k),X(:,k-1));
    X(:,k)=X(:,k-1)+dX*0.01;
    fhandle(:,k)=handle;
    fapplied(:,k)=applied';
end

X=X';