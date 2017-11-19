function [orientationS,angVS,angA,angAS,accel,accelS] = get_smooth_imu(orientation,Rot_imu,angV,araw,time,steering,throttle,rctime)
%input 3xN (roll,pitch,yaw) orientation from imu, angular velocities, raw acceleration data

%output smooth orientation, angular velocity, calibrated acceleration 3xN
%unwrapped but not smooth heading
%calibrated but not smooth acceleration
b=-0.0543;
a=0.048;

orientation=[1*ones(1,length(time));-1*ones(1,length(time));-1*ones(1,length(time))].*orientation;
headingU=unwrap(orientation(3,:));
headingU=filterdiff(headingU,time,0.25);
orientationS=smooth_3([orientation(1,:);orientation(2,:);headingU],time);

angV=[1*ones(1,length(time));-1*ones(1,length(time));-1*ones(1,length(time))].*angV;
angVS=smooth_3(angV,time);

accel=zeros(3,length(time));
for i=1:length(time)
  accel(:,i)=araw(:,i)-Rot_imu{i}'*[0;0;9.80665];
end

angA=get_dt(angVS,time);
angA=smooth_3(angA,time);
angAS=smooth_accel(angA,time,steering,throttle,rctime,0,.1,30);

accel=[1*ones(1,length(time));-1*ones(1,length(time));-1*ones(1,length(time))].*araw;
trans=[b*angA(3,:)+a*angVS(3,:).^2;-a*angA(3,:)+b*angVS(3,:).^2;zeros(1,length(time))];
accel=accel+trans;
accel=smooth_3(accel,time);
accelS=smooth_accel(accel,time,steering,throttle,rctime,0,.1,30);


end

