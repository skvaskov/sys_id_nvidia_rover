function [velocity] = get_dt(x,time)
%positions 3XNtimesteps 
sz=size(x);
velocity=zeros(sz);
dS=x(:,2)-x(:,1);
dt=time(2)-time(1);
velocity(:,1)=dS/dt;
for i=2:sz(2)-1
dS=x(:,i+1)-x(:,i-1);
dt=time(i+1)-time(i-1);
velocity(:,i)=dS/dt;
end
dS=x(:,end)-x(:,end-1);
dt=time(end)-time(end-1);
velocity(:,end)=dS/dt;
end

