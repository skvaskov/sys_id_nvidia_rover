%start with this, you will need to modify what spreadsheets it pulls data
%from, but it basically reads in excel files and saves the data in .mat
%workspace

clc
clear all
close all
trialnames={'speed1_11_28','turns_11_28'};
for it=1:length(trialnames)
clearvars -except it trialnames 
%enter file path to place where raw data is stored
%enter trial name
trialname=trialnames{it};
full=['~/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Raw_Data/',trialname];
%% motor
%load motor data
motorstates_name=[full,'_pololu_motor_states.csv'];
motorstates_excel=readtable(motorstates_name);
time_motorstates=table2array(motorstates_excel(:,1));
time_motorstates=time_motorstates*10^(-9);
steering_pulse=table2array(motorstates_excel(:,19));
throttle_pulse=table2array(motorstates_excel(:,6));

clearvars motorstates_name motorstates_excel

%determine startpoint by positive throttle
start_motorstates=find(throttle_pulse-1500>=0,1)-1;

%load command data
command_name=[full,'_pololu_command.csv'];
command_excel=readtable(command_name);
steervals=logical(table2array(command_excel(:,2)));
time_steering_command=table2array(command_excel(steervals,1));
time_steering_command=time_steering_command*10^(-9);
steering_command=table2array(command_excel(steervals,3));
time_throttle_command=table2array(command_excel(~steervals,1));
time_throttle_command=time_throttle_command*10^(-9);
throttle_command=table2array(command_excel(~steervals,3));

clearvars steervals command_name command_excel sz even odd

%determine startpoint by positive throttle
start_command=find(-throttle_command>=0,1)-1;
%% imu
% %load imu data
imuname=[full,'_imu_imu.csv'];
if exist(imuname,'file')


imuexcel=readtable(imuname);
time_imu=table2array(imuexcel(:,5))+table2array(imuexcel(:,6))*10^(-9);
accel_imu=table2array(imuexcel(:,20:22))';
angV_imu=table2array(imuexcel(:,15:17))';

orientI=table2array(imuexcel(:,9));
orientJ=table2array(imuexcel(:,10));
orientK=table2array(imuexcel(:,11));
orientW=table2array(imuexcel(:,12));

orientation_imu=zeros([3,length(time_imu)]);
Rot_imu=cell([1,length(time_imu)]);
for i=1:length(time_imu)
    [Rot_imu{i},orientation_imu(:,i)]=qGetR([orientW(i) orientI(i) orientJ(i) orientK(i)]);
end



start_imu=get_start(accel_imu(1,:),time_imu,.1,.25,5);
end
clearvars imuname imuexcel imustamp orientI orientJ orientK orientW
%% mocap
% %load mocap data

moname =[full,'_mocap.csv'];
moexcel=readtable(moname);

time_mocap=table2array(moexcel(:,5))+table2array(moexcel(:,6))*10^(-9);

I=table2array(moexcel(:,12));
K=table2array(moexcel(:,11));
J=table2array(moexcel(:,10));

J=smooth(time_mocap,J)/1000;
K=smooth(time_mocap,K)/1000;
I=smooth(time_mocap,I)/1000;

orientW=table2array(moexcel(:,17));
orientJ=table2array(moexcel(:,14));
orientK=table2array(moexcel(:,15));
orientI=table2array(moexcel(:,16));


orientation_mocap=zeros([3,length(time_mocap)]);
Rot_mocap=cell([1,length(time_mocap)]);
for i=1:length(time_mocap)
    [Rot_mocap{i},orientation_mocap(:,i)]=qGetR([orientW(i) orientI(i) orientJ(i) orientK(i)]);
end


CenterPos=[I';J';K'];
CenterPosS=smooth_3(CenterPos,time_mocap);
if length(time_mocap)>3000
v=get_velocity(CenterPosS(:,1:3000),Rot_mocap,time_mocap(1:3000));
else
 v=get_velocity(CenterPosS,Rot_mocap,time_mocap);   
end

start_mocap=get_start(v(1,:),time_mocap,.2,2,60);

clearvars moexcel moname  v orientW orientI orientJ orientK I J K i




save([trialnames{it},'_workspace.mat'],'-regexp','^(?!(it|trialnames|full)$).')


end
    

