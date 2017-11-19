function [CPosS,orientationS,velS,angVS,angA,angAS,accel,accelS] = get_smooth_mocap(CenterPos,cmoffset,orientation,Rot,time,steering,throttle,rctime)
%inputs
%3xN position (x;y;z),
%3x1 center of mass offset (center to center of mass)
%3XN orientation
%1xN cell array of 3x3 rotation matrices 
%Nx1 time steps for mocap data
%Nx1 steering input
%Nx1 throttle input
%Nx1 time vector for inputs

%smooth position, unwrap and smooth heading
 
headingU=unwrap(orientation(3,:));
headingU=filterdiff(headingU,time,0.25);
orientationS=smooth_3([orientation(1,:);orientation(2,:);headingU],time);


CPosS=centroid_topoint(CenterPos,Rot,cmoffset);


%get velocities
angV=get_dt(orientationS,time);
angV=[filterdiff(angV(1,:),time,.1);filterdiff(angV(2,:),time,.1);filterdiff(angV(3,:),time,.1)];
angVS=smooth_3(angV,time);

vel=get_velocity(CPosS,Rot,time);
vel=[filterdiff(vel(1,:),time,.1);filterdiff(vel(2,:),time,.05);filterdiff(vel(2,:),time,.1)];
velS=smooth_3(vel,time);
    


%get accelerations
accel=get_dt(velS,time);
accel=smooth_3(accel,time);
accelS=smooth_accel(accel,time,steering,throttle,rctime,0,.1,30);
angA=get_dt(angVS,time);
angA=smooth_3(angA,time);
angAS=smooth_accel(angA,time,steering,throttle,rctime,0,.1,30);





end

