function [u0,u1,vx,ax,vy,w,delta,delta_dot,t] = pwm_mocap_points(data,Pveh,velocity_threshold,throttle_threshold,inputplot)
%input:
%data:struture with trials to use
%Pveh: vehicle paramters Pveh=[m,Iz,l,lr];
%velocity_threshold: [minimum velocity, maximum velocity]
%command_threshold: [minimum command, maximum command]

u0=[];
vx=[];
ax=[];
vy=[];
w=[];
delta=[];
u1=[];
delta_dot=[];
t=[];
for i=1:length(data)
indexes=(data(i).interp.mocap.velocity(1,:)>=velocity_threshold(1)) & (data(i).interp.mocap.velocity(1,:)<=velocity_threshold(2))...
    & (data(i).interp.input.command.throttle>=throttle_threshold(1)) & (data(i).interp.input.command.throttle<=throttle_threshold(2))&(~isnan(data(i).interp.input.command.steering));
if find(indexes,1)
u0=[u0;data(i).interp.input.command.throttle(indexes)'];
u1=[u1;data(i).interp.input.command.steering(indexes)'];
vx=[vx;data(i).interp.mocap.velocity(1,indexes)'];
vy=[vy;data(i).interp.mocap.velocity(2,indexes)'];
ax=[ax;data(i).interp.mocap.local_accel_smooth(1,indexes)'];
w=[w;data(i).interp.mocap.angular_velocity(3,indexes)'];
dadd=atan(Pveh(3)*data(i).interp.mocap.angular_velocity(3,indexes)./data(i).interp.mocap.velocity(1,indexes));
time=data(i).interp.time(indexes);
t=[t;time'];
dadd=filterdiff(dadd,time,2);
delta=[delta;smooth(time,dadd)];
ddotadd=get_dt(dadd',time)';
delta_dot=[delta_dot;smooth(time,ddotadd)];


if inputplot
figure(1)
hold on
plot(time,data(i).interp.input.command.throttle(indexes)')
title('trained inputs')
end
else
    disp(['discard trial: ',num2str(i)])
end

clearvars indexes
end
%delta=atan(Pveh(3)*w./vx);
% ddes=0.224314009055080*u1- 0.008867066788855;
% delta_dot=4.300730919846748*(ddes-delta);

end

