
%loads a file that has already been saved after running the workspace and
%has been lined up timewise. This saves a data structure with all of the
%different trials, it tries to break things up to eliminate motion where i
%am picking up the rover

clear all
close all
clc
data=struct('name',{},'no',{},'interp',{});
wnames={'joy1_11_17_workspace.mat','joy2_11_17_workspace.mat'};
trialname='joy';
%% MANUALLY ENTER THE FOLLOWING VALUES
%distance from center of mocap data to cetner of mass
cmassoffset=[-.009;0;0];


%%

for num=1:length(wnames)
 clearvars -except data wnames trialname cmassoffset num
load(wnames{num});    
shiftedtime_imu=time_imu-shift_imu;
shiftedtime_motorstates=time_motorstates-shift_motorstates;
shiftedtime_throttle_command=time_throttle_command-shift_command;
shiftedtime_steering_command=time_steering_command-shift_command;
shiftedtime_mocap=time_mocap-shift_mocap;

%get time
time=shiftedtime_mocap;

%combine steering and throttle commands on one time vector
shiftedtime_command_combined=unique(sort([shiftedtime_steering_command;shiftedtime_throttle_command]));
throttle_command_combined=zoh(shiftedtime_throttle_command,throttle_command,shiftedtime_command_combined);
steering_command_combined=zoh(shiftedtime_steering_command,steering_command,shiftedtime_command_combined);

%mocapdat
[CPosS_mocap,orientationS_mocap,velS_mocap,angVS_mocap,angA_mocap,angAS_mocap,accel_mocap,accelS_mocap] = ...
    get_smooth_mocap(CenterPosS,cmassoffset,orientation_mocap,Rot_mocap,shiftedtime_mocap,steering_command_combined,throttle_command_combined,shiftedtime_command_combined);

%store mocapdata
data(num).no.mocap.pos=CPosS_mocap;
data(num).no.mocap.orientation=orientationS_mocap;
data(num).no.mocap.angular_velocity=angVS_mocap;

data(num).no.mocap.velocity=velS_mocap;

data(num).no.mocap.angular_accel=angA_mocap;
data(num).no.mocap.angular_accel_smooth=angAS_mocap;

data(num).no.mocap.local_accel=accel_mocap;
data(num).no.mocap.local_accel_smooth=accelS_mocap;

data(num).no.mocap.time=shiftedtime_mocap'-time(1);


[orientationS_imu,angVS_imu,angA_imu,angAS_imu,accel_imu,accelS_imu] = ...
    get_smooth_imu(orientation_imu,Rot_imu,angV_imu,accel_imu,shiftedtime_imu,steering_command_combined,throttle_command_combined,shiftedtime_command_combined);

data(num).no.imu.global_accel=accel_imu;
data(num).no.imu.global_accel_smooth=accelS_imu;

data(num).no.imu.angular_velocity=angVS_imu;

data(num).no.imu.angular_accel=angA_imu;
data(num).no.imu.angular_accel_smooth=angAS_imu;

data(num).no.imu.orientation=orientationS_imu;

data(num).no.imu.time=shiftedtime_imu'-time(1);



%commandinput

data(num).no.input.command.time=shiftedtime_command_combined-time(1);
data(num).no.input.command.throttle=throttle_command_combined;
data(num).no.input.command.steering=steering_command_combined;

data(num).no.input.pulse.time=shiftedtime_motorstates'-time(1);
data(num).no.input.pulse.throttle=throttle_pulse;
data(num).no.input.pulse.steering=steering_pulse;




%store interpolated data
data(num).name=[trialname,': ',num2str(num)];
data(num).interp.time=time'-time(1);

%mocap
data(num).interp.mocap.pos=interp1(shiftedtime_mocap,CPosS_mocap',time)';

data(num).interp.mocap.orientation=interp1(shiftedtime_mocap,orientationS_mocap',time)';

data(num).interp.mocap.velocity=interp1(shiftedtime_mocap,velS_mocap',time)';
data(num).interp.mocap.angular_velocity=interp1(shiftedtime_mocap,angVS_mocap',time)';
  
data(num).interp.mocap.local_accel=interp1(shiftedtime_mocap,accel_mocap',time)';
data(num).interp.mocap.local_accel_smooth=interp1(shiftedtime_mocap,accelS_mocap',time)';


data(num).interp.mocap.angular_accel=interp1(shiftedtime_mocap,angA_mocap',time)';
data(num).interp.mocap.angular_accel_smooth=interp1(shiftedtime_mocap,angAS_mocap',time)'; 

%input
data(num).interp.input.command.throttle=interp1(shiftedtime_command_combined,throttle_command_combined,time)';
data(num).interp.input.command.steering=interp1(shiftedtime_command_combined,steering_command_combined,time)';
data(num).interp.input.pulse.throttle=interp1(shiftedtime_motorstates,throttle_pulse,time)';
data(num).interp.input.pulse.steering=interp1(shiftedtime_motorstates,steering_pulse,time)';
%imu
data(num).interp.imu.orientation=interp1(shiftedtime_imu,orientationS_imu',time)';
data(num).interp.imu.angular_velocity=interp1(shiftedtime_imu,angVS_imu',time)';

data(num).interp.imu.angular_accel=interp1(shiftedtime_imu,angA_imu',time)';
data(num).interp.imu.angular_accel_smooth=interp1(shiftedtime_imu,angAS_imu',time)'; 


data(num).interp.imu.global_accel=interp1(shiftedtime_imu,accel_imu',time)';
data(num).interp.imu.global_accel_smooth=interp1(shiftedtime_imu,accelS_imu',time)';


end


