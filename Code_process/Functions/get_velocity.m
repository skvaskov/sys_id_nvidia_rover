function [velocity] = get_velocity(CPos,R,time)
%positions 3XNtimesteps 
sz=size(CPos);
velocity=zeros(sz);
dS=CPos(:,2)-CPos(:,1);
dt=time(2)-time(1);
velocity(:,1)=R{1}'*dS/dt;
for i=2:sz(2)-1
dS=CPos(:,i+1)-CPos(:,i);
dt=time(i+1)-time(i);
velocity(:,i)=R{i}'*dS/dt;
end
dS=CPos(:,end)-CPos(:,end-1);
dt=time(end)-time(end-1);
velocity(:,end)=R{end}'*dS/dt;

end

