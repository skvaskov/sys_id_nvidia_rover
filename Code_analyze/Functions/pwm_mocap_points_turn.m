function [P_out,vx_out,ax_out,vy_out,w_out,Pst_out] = pwm_mocap_points_turn(data,tnums,threshold,pl,Pth,vx,ax,vy,w,Pst)
%input:structure of data 
%trialnums: numbers in data structure to use
%threshold: minimum velocity for data points
%pl 1 if you want to plot data
%output ax: acceleration, vx: velocity, P: throttle channel in colum
%vectors
ax_out=ax;
P_out=Pth;
Pst_out=Pst;
vx_out=vx;
vy_out=vy;
w_out=w;
for i=1:length(tnums)

idxs=find(data(tnums(i)).interp.mocap.speed>=threshold,1);
idxe=find(data(tnums(i)).interp.mocap.speed>=threshold,1,'last');

ax_out=[ax_out;data(tnums(i)).interp.mocap.longaccel_smooth(idxs:idxe)];
 vx_out=[vx_out;data(tnums(i)).interp.mocap.longvelocity_smooth(idxs:idxe)];
 vy_out=[vy_out;data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe)];
 w_out=[w_out;data(tnums(i)).interp.mocap.yawrate_smooth(idxs:idxe)];
% vel=[data(tnums(i)).interp.mocap.longvelocity_smooth(idxs:idxe),data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe),zeros(size(data(tnums(i)).interp.mocap.latvelocity_smooth(idxs:idxe)))]';
% t=data(tnums(i)).interp.time(idxs:idxe);
% yr=smooth_accel1(data(tnums(i)).interp.mocap.yawrate_smooth(idxs:idxe),t,data(tnums(i)).interp.input.steering(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe),t,0,.1,50);
% w_out=[w_out;yr];
% velS=smooth_accel(vel,t,data(tnums(i)).interp.input.steering(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe),t,0,.1,50);
% vx_out=[vx_out;velS(1,:)'];
%  vy_out=[vy_out;velS(2,:)'];
P_out=[P_out;data(tnums(i)).interp.input.throttle(idxs:idxe)];
Pst_out=[Pst_out;data(tnums(i)).interp.input.steering(idxs:idxe)];   
if pl
figure(1)
hold on
plot(data(tnums(i)).interp.time(idxs:idxe),data(tnums(i)).interp.input.throttle(idxs:idxe))
title('trained inputs')
end
end

end

