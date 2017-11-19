function [P_out,vx_out,ax_out,vy_out,w_out,P_stout]= pwm_imu_points_turn(data,tnums,threshold,pl,P,vx,ax,vy,w,Pst)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points

%output ax: acceleration, vx: velocity, P: throttle channel in colum
%vectors
axadd=ax;
Padd=P;
vxadd=vx;
Pstadd=Pst;
vyadd=vy;
wadd=w;
for i=1:length(tnums)

idxs=find(1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth+data(tnums(i)).interp.encoder.right_smooth)/2>=threshold,1);
idxe=find(1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth+data(tnums(i)).interp.encoder.right_smooth)/2>=threshold,1,'last');
vyadd=[vyadd;data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe)];
wadd=[wadd;data(tnums(i)).interp.imu.yawrate_smooth(idxs:idxe)];
axadd=[axadd;data(tnums(i)).interp.imu.longaccel(idxs:idxe)];
vxadd=[vxadd;1.3/1.193*(data(tnums(i)).interp.encoder.left_smooth(idxs:idxe)+data(tnums(i)).interp.encoder.right_smooth(idxs:idxe))/2];
Padd=[Padd;data(tnums(i)).interp.input.throttle(idxs:idxe)];
Pstadd=[Pstadd;data(tnums(i)).interp.input.steering(idxs:idxe)]; 

if pl
figure(1)
hold on
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe))
title('trained inputs')
end
end
P_out=Padd;
ax_out=axadd;
vx_out=vxadd;
vy_out=vyadd;
w_out=wadd;
P_stout=Pstadd;


end