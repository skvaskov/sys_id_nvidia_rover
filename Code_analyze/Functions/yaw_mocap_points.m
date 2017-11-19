function [delta_out,throttle_out,vx_out,vy_out,w_out,ay_out,aw_out] = yaw_mocap_points(data,tnums,threshold,pl,sfd)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points
%pl 1 if you want to plot data
%output ax: acceleration, vx: velocity, delta steering angle 
delta_out=[];
ay_out=[];
aw_out=[];
w_out=[];
vy_out=[];
vx_out=[];
throttle_out=[];

for i=1:length(tnums)

idxs=find(data(tnums(i)).interp.mocap.speed>=threshold,1);
idxe=find(data(tnums(i)).interp.mocap.speed>=threshold,1,'last');

ay_out=[ay_out;data(tnums(i)).interp.mocap.lataccel_smooth(idxs:idxe)];
aw_out=[aw_out;data(tnums(i)).interp.mocap.yawaccel_smooth(idxs:idxe)];
%w_out=[w_out;data(tnums(i)).interp.imu.yawrate_smooth(idxs:idxe)];

vel=[data(tnums(i)).interp.mocap.longvelocity_smooth(idxs:idxe),data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe),zeros(size(data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe)))]';
t=data(tnums(i)).interp.time(idxs:idxe);
yr=smooth_accel1(data(tnums(i)).interp.mocap.yawrate_smooth(idxs:idxe),t,data(tnums(i)).interp.input.steering(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe),t,0,.1,50);
w_out=[w_out;yr];
velS=smooth_accel(vel,t,data(tnums(i)).interp.input.steering(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe),t,0,.1,50);
% vx_out=[vx_out;data(tnums(i)).interp.mocap.longvelocity_smooth(idxs:idxe)];
% vy_out=[vy_out;data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe)];
vx_out=[vx_out;velS(1,:)'];
 vy_out=[vy_out;velS(2,:)'];
delta_add=steeringfit(data(tnums(i)).interp.input.steering(idxs:idxe),sfd);
delta_out=[delta_out;delta_add];
throttle_out=[throttle_out;data(tnums(i)).interp.input.throttle(idxs:idxe)];    
if pl
figure(1)
hold on
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.steering(idxs:idxe),'m')
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.mocap.longvelocity_smooth(idxs:idxe),'g')
title('trained inputs')
end

end

% vx_out=vx_out';
% vy_out=vy_out';
end

