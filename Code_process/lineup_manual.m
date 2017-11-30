%after making a workspace i run this to ensure the timestamps for
%everything is lined up

clear all
close all
clc

tname='turns_11_29_workspace.mat';
load(tname)
%% select range of each data type (motor state, motor command, imu, mocap) to view
imusp=1:4000;
mocapsp=1:1500;
pulsesp=1:1500;
commandsp=1:28;



%%
if exist('accel_imu','var')
accxbias=mean(accel_imu(1,1:500));
accybias=mean(accel_imu(2,1:500));
end
vsp=get_velocity(CenterPosS(:,mocapsp),Rot_mocap,time_mocap(mocapsp));
vspS=smooth_3(vsp,time_mocap(mocapsp));


yawratesp=get_dt(smooth(time_mocap(mocapsp),orientation_mocap(3,mocapsp))',time_mocap(mocapsp));
yawratespS=smooth(time_mocap(mocapsp),yawratesp);
accelsp=get_dt(vspS,time_mocap(mocapsp));
accelspS=[smooth(time_mocap(mocapsp),accelsp(1,:))';smooth(time_mocap(mocapsp),accelsp(2,:))';smooth(time_mocap(mocapsp),accelsp(3,:))'];

%% MANUALLY ENTER THE FOLLOWING VALUES
if exist('accel_imu','var')
shift_imu=time_imu(start_imu+1);
end
shift_command=time_throttle_command(start_command+1);
shift_motorstates=time_motorstates(start_motorstates+1)+9.342;
shift_mocap=time_mocap(start_mocap);

if exist('accel_imu','var')
shiftedtime_imu=time_imu-shift_imu;
end
shiftedtime_command=time_throttle_command-shift_command;
shiftedtime_motorstates=time_motorstates-shift_motorstates;
shiftedtime_mocap=time_mocap-shift_mocap;

figure
subplot(3,1,1)
yyaxis('left')
plot(shiftedtime_mocap(mocapsp),vspS(1,:),'b-')
hold on
plot(shiftedtime_command(commandsp),throttle_command(commandsp))
ylim([-1,3])
hold off
yyaxis('right')
plot(shiftedtime_motorstates(pulsesp),throttle_pulse(pulsesp),'g-')
hold off
subplot(3,1,2)
yyaxis('left')
plot(shiftedtime_mocap(mocapsp),accelsp(1,:),'b-')
hold on
if exist('accel_imu','var')
plot(shiftedtime_imu(imusp),accel_imu(1,imusp),'r-')
end
plot(shiftedtime_command(commandsp),throttle_command(commandsp))
ylim([-5 5])
hold off
yyaxis('right')
plot(shiftedtime_motorstates(pulsesp),throttle_pulse(pulsesp),'g-')
hold off
subplot(3,1,3)
plot(shiftedtime_mocap(mocapsp),yawratespS','b');
hold on
plot(shiftedtime_command(commandsp),steering_command(commandsp),'g-')
if exist('accel_imu','var')
plot(shiftedtime_imu(imusp),angV_imu(3,imusp),'r')
end
ylim([-3 3])
%%
sv=input('enter 521 if you want to save the shift values: ')
if exist('accel_imu','var')
    if sv==521
    save(tname,'shift_*','*bias','-append')
    disp('ok')
    end
else
    if sv==521
    save(tname,'shift_*','-append')
    disp('ok')
    end
end
