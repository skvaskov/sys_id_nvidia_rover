
%loads a file that has already been saved after running the workspace and
%has been lined up timewise. This saves a data structure with all of the
%different trials, it tries to break things up to eliminate motion where i
%am picking up the rover

clear all
close all
clc
data=struct('name',{},'no',{},'interp',{});
load('turns_11_29_workspace.mat')
trialname='turn_1';
%% MANUALLY ENTER THE FOLLOWING VALUES
%distance from center of mocap data to cetner of mass
cmassoffset=[-.009;0;0];
%%
if exist('time_imu','var')
shiftedtime_imu=time_imu-shift_imu;
end
shiftedtime_motorstates=time_motorstates-shift_motorstates;
shiftedtime_throttle_command=time_throttle_command-shift_command;
shiftedtime_steering_command=time_steering_command-shift_command;
shiftedtime_mocap=time_mocap-shift_mocap;


%combine steering and throttle commands on one time vector
shiftedtime_command_combined=unique(sort([shiftedtime_steering_command;shiftedtime_throttle_command]));
throttle_command_combined=zoh(shiftedtime_throttle_command,throttle_command,shiftedtime_command_combined);
steering_command_combined=zoh(shiftedtime_steering_command,steering_command,shiftedtime_command_combined);

%%
eq=throttle_command==0;
diffeq=abs(diff(eq));
idxswitch=find(diffeq);
if idxswitch(1)~=1
    idxswitch=[1;idxswitch];
end
ln=length(idxswitch)/2;
for num=1:ln
idxs_start=idxswitch(2*num-1)+1;
idxs_end=idxswitch(2*num);
time_start=shiftedtime_throttle_command(idxs_start)-0.25;
time_end=shiftedtime_throttle_command(idxs_end);


idxs_start_mocap=find(shiftedtime_mocap>=time_start,1);
idxs_end_mocap=find(shiftedtime_mocap<=time_end,1,'last');
if exist('time_imu','var')
idxs_start_imu=find(shiftedtime_imu>=time_start,1);
idxs_end_imu=find(shiftedtime_imu<=time_end,1,'last');
else
    idxs_start_imu=[];
end
idxs_start_command=find(shiftedtime_command_combined>=time_start,1);
idxs_end_command=find(shiftedtime_command_combined<=time_end,1,'last');

idxs_start_motorstates=find(shiftedtime_motorstates>=time_start,1);
idxs_end_motorstates=find(shiftedtime_motorstates<=time_end,1,'last');



%define mocap cuts
positioncut=CenterPosS(:,idxs_start_mocap:idxs_end_mocap);
orientationmocapcut=orientation_mocap(:,idxs_start_mocap:idxs_end_mocap);
Rotmocapcut=Rot_mocap(idxs_start_mocap:idxs_end_mocap);
mocaptimecut=shiftedtime_mocap(idxs_start_mocap:idxs_end_mocap);

%define cuts for pulse data to be used in mocap
commandtimecut=shiftedtime_command_combined(idxs_start_command:idxs_end_command);
throttlecut=throttle_command_combined(idxs_start_command:idxs_end_command);
steeringcut=steering_command_combined(idxs_start_command:idxs_end_command);



if ~isempty(orientationmocapcut)
%mocapdat
[CPosS_mocap_cut,orientationS_mocap_cut,velS_mocap_cut,angVS_mocap_cut,angA_mocap_cut,angAS_mocap_cut,accel_mocap_cut,accelS_mocap_cut] = ...
    get_smooth_mocap(positioncut,cmassoffset,orientationmocapcut,Rotmocapcut,mocaptimecut,steeringcut,throttlecut,commandtimecut);



%select time for itnerpolation
time=mocaptimecut;


%store mocapdata
data(num).no.mocap.pos=CPosS_mocap_cut;
data(num).no.mocap.orientation=orientationS_mocap_cut;
data(num).no.mocap.angular_velocity=angVS_mocap_cut;

data(num).no.mocap.velocity=velS_mocap_cut;

data(num).no.mocap.angular_accel=angA_mocap_cut;
data(num).no.mocap.angular_accel_smooth=angAS_mocap_cut;

data(num).no.mocap.local_accel=accel_mocap_cut;
data(num).no.mocap.local_accel_smooth=accelS_mocap_cut;

data(num).no.mocap.time=mocaptimecut'-time(1);

%to define imu cuts
if ~isempty(idxs_start_imu)
orientationimucut=orientation_imu(:,idxs_start_imu:idxs_end_imu);
Rotimucut=Rot_imu(idxs_start_imu:idxs_end_imu);
angVimucut=angV_imu(:,idxs_start_imu:idxs_end_imu);
imutimecut=shiftedtime_imu(idxs_start_imu:idxs_end_imu);
accelcut=accel_imu(:,idxs_start_imu:idxs_end_imu);

[orientationS_imu_cut,angVS_imu_cut,angA_imu_cut,angAS_imu_cut,accel_imu_cut,accelS_imu_cut] = ...
    get_smooth_imu(orientationimucut,Rotimucut,angVimucut,accelcut,imutimecut,steeringcut,throttlecut,commandtimecut);

data(num).no.imu.measured_accel=accel_imu_cut;
data(num).no.imu.measured_accel_smooth=accelS_imu_cut;

data(num).no.imu.angular_velocity=angVS_imu_cut;

data(num).no.imu.angular_accel=angA_imu_cut;
data(num).no.imu.angular_accel_smooth=angAS_imu_cut;

data(num).no.imu.orientation=orientationS_imu_cut;

data(num).no.imu.time=imutimecut'-time(1);
end
%commandinput

data(num).no.input.command.time=commandtimecut'-time(1);
data(num).no.input.command.throttle=throttlecut;
data(num).no.input.command.steering=steeringcut;

data(num).no.input.pulse.time=shiftedtime_motorstates(idxs_start_motorstates:idxs_end_motorstates)'-time(1);
data(num).no.input.pulse.throttle=throttle_pulse(idxs_start_motorstates:idxs_end_motorstates);
data(num).no.input.pulse.throttle=steering_pulse(idxs_start_motorstates:idxs_end_motorstates);



%store interpolated data
data(num).name=[trialname,': ',num2str(num)];
data(num).interp.time=time'-time(1);

%mocap
data(num).interp.mocap.pos=interp1(mocaptimecut,CPosS_mocap_cut',time)';

data(num).interp.mocap.orientation=interp1(mocaptimecut,orientationS_mocap_cut',time)';

data(num).interp.mocap.velocity=interp1(mocaptimecut,velS_mocap_cut',time)';
data(num).interp.mocap.angular_velocity=interp1(mocaptimecut,angVS_mocap_cut',time)';
  
data(num).interp.mocap.local_accel=interp1(mocaptimecut,accel_mocap_cut',time)';
data(num).interp.mocap.local_accel_smooth=interp1(mocaptimecut,accelS_mocap_cut',time)';


data(num).interp.mocap.angular_accel=interp1(mocaptimecut,angA_mocap_cut',time)';
data(num).interp.mocap.angular_accel_smooth=interp1(mocaptimecut,angAS_mocap_cut',time)'; 

%input
data(num).interp.input.command.throttle=zoh(commandtimecut,throttlecut,time)';
data(num).interp.input.command.steering=zoh(commandtimecut,steeringcut,time)';
data(num).interp.input.pulse.throttle=interp1(shiftedtime_motorstates(idxs_start_motorstates:idxs_end_motorstates),throttle_pulse(idxs_start_motorstates:idxs_end_motorstates),time)';
data(num).interp.input.pulse.steering=interp1(shiftedtime_motorstates(idxs_start_motorstates:idxs_end_motorstates),throttle_pulse(idxs_start_motorstates:idxs_end_motorstates),time)';
%imu
if ~isempty(idxs_start_imu)
data(num).interp.imu.orientation=interp1(imutimecut,orientationS_imu_cut',time)';
data(num).interp.imu.angular_velocity=interp1(imutimecut,angVS_imu_cut',time)';

data(num).interp.imu.angular_accel=interp1(imutimecut,angA_imu_cut',time)';
data(num).interp.imu.angular_accel_smooth=interp1(imutimecut,angAS_imu_cut',time)'; 


data(num).interp.imu.measured_accel=interp1(imutimecut,accel_imu_cut',time)';
data(num).interp.imu.measured_accel_smooth=interp1(imutimecut,accelS_imu_cut',time)';
end
end
end


