function [delta_out,th_out,vx_out,vy_out,w_out,ay_out,aw_out] = yaw_imu_points(data,tnums,threshold,pl,deg)
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
th_out=[];
for i=1:length(tnums)

idxs=find(data(tnums(i)).interp.mocap.speed>=threshold,1);
idxe=find(data(tnums(i)).interp.mocap.speed>=threshold,1,'last');

ay_out=[ay_out;data(tnums(i)).interp.mocap.lataccel_smooth(idxs:idxe)];
aw_out=[aw_out;data(tnums(i)).interp.imu.yawaccel_smooth(idxs:idxe)];
ax=data(tnums(i)).interp.mocap.longaccel_smooth(idxs:idxe);
w_out=[w_out;data(tnums(i)).interp.imu.yawrate_smooth(idxs:idxe)];
vx_out=[vx_out;1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth(idxs:idxe)+data(tnums(i)).interp.encoder.left_smooth(idxs:idxe))/2];
vya=(ax-data(tnums(i)).interp.mocap.longaccel_smooth(idxs:idxe)).*(data(tnums(i)).interp.imu.yawrate_smooth(idxs:idxe).^-1);
vy_out=[vy_out;vya];
delta_add=steeringfit(data(tnums(i)).interp.input.steering(idxs:idxe),deg);
delta_out=[delta_out;delta_add];
th_out=[th_out;data(tnums(i)).interp.input.throttle(idxs:idxe)];
if pl
figure(1)
hold on
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.steering(idxs:idxe),'m')
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe),'g')
title('trained inputs')
end

end


end

