load('data_circles_rot.mat')
data=data(8);
clearvars -except data

a_raw=data.no.imu.measured_accel;
sz=size(a_raw);
Rot=data.no.imu.Rot;
a_cal=zeros(size(a_raw));
for i=1:sz(2)
    a_cal(:,i)=a_raw(:,i)+Rot{i}'*[0;0;9.80655];
end


figure(1)
for i=1:3
subplot(3,1,i)
plot(data.no.imu.time,a_cal(i,:))
hold on
end

%mocap data
a=0.048;
b=-0.0543;
lr=0.3302;

vx_dot=data.no.mocap.local_accel_smooth(1,:);
vy_dot=data.no.mocap.local_accel_smooth(2,:);
vx=data.no.mocap.velocity(1,:);
vy=data.no.mocap.velocity(2,:);
w=data.no.mocap.angular_velocity(3,:);
w_dot=get_dt(w,data.no.mocap.time);
w_dot=smooth(data.no.mocap.time,w_dot)';

%data computed using all data
ax_mocap=-a*w_dot-b*w.^2+(vx_dot-vy.*w);
ay_mocap=-(b*w_dot-a*w.^2+(vy_dot+vx.*w));

%data computing using model
ax_mocap=-a*w_dot-b*w.^2+(vx_dot-lr*w.*w);
ay_mocap=-(b*w_dot-a*w.^2+(lr*w_dot+vx.*w));

subplot(3,1,1)
plot(data.no.mocap.time,ax_mocap)
subplot(3,1,2)
plot(data.no.mocap.time,ay_mocap)

%find covariance
idxs=400:800;
ax_meas=mean(a_cal(1,idxs));
ay_meas=mean(a_cal(2,idxs));
ax_meas=ax_meas*ones(size(a_cal(1,idxs)));
ay_meas=ay_meas*ones(size(a_cal(2,idxs)));
err_imu=[ax_meas',ay_meas']-a_cal(1:2,idxs)';
covar_imu =  err_imu.' * err_imu;
    
 covar_imu = covar_imu / (length(data.no.imu.time)-1)
