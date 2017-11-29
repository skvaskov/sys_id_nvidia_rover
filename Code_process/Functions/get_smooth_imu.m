function [orientationS,angVS,angA,angAS,accel,accelS] = get_smooth_imu(orientation,Rot_imu,angV,araw,time,steering,throttle,rctime)
%input 3xN (roll,pitch,yaw) orientation from imu, angular velocities, raw acceleration data

%output smooth orientation, angular velocity, calibrated acceleration 3xN
%unwrapped but not smooth heading
%calibrated but not smooth acceleration


headingU=unwrap(orientation(3,:));
headingU=filterdiff(headingU,time,0.25);
orientationS=smooth_3([orientation(1,:);orientation(2,:);headingU],time);


angVS=smooth_3(angV,time);


angA=get_dt(angVS,time);
angA=smooth_3(angA,time);
angAS=smooth_accel(angA,time,steering,throttle,rctime,0,.1,30);


accel=smooth_3(araw,time);
accelS=smooth_accel(accel,time,steering,throttle,rctime,0,.1,30);


end

