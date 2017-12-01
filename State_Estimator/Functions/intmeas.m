function [x] = intmeas(dxdt,time)
x=zeros(size(time));
fildxdt=zeros(size(dxdt));
fildxdt(1)=dxdt(1);
for i=2:length(fildxdt)
fildxdt(i)=.9*fildxdt(i-1)+.1*dxdt(i);
end
for i=2:length(x)
    x(i)=x(i-1)+dxdt(i-1)*(time(i)-time(i-1));
end
end

