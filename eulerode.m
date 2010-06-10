function [T,X]=eulerode(fun, time, ini)

T=time(1):0.01:time(end);
X=zeros(length(ini),length(T));
X(:,1)=ini;

for k=2:length(T)
    X(:,k)=X(:,k-1)+fun(T(k),X(:,k-1))*0.01;
end

X=X';