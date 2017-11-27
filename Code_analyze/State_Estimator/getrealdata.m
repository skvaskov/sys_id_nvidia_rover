%
clear all
close all
clc

load('/Users/seanvaskov/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Code_analyze/Processed_Data/steeringdata.mat')
trial=sturns(4);
path=['/Users/seanvaskov/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Code_analyze/State_Estimator'];
name=['real_sturn'];

%%
motime=trial.no.mocap.time;
imutime=trial.no.imu.time;
enctime=trial.no.encoder.time;
%comptime=trial.no.compass.time;
intime=trial.no.input.time;

starttime=intime(1);
endtime=intime(end);

%write csv file for trajectory
T=motime';
%startmocap at 0 heading
theta=mean(trial.no.mocap.heading_unwrapped_smooth(1:5));
ipos=mean(trial.no.mocap.pos(1:2,1:5),2);
pos= [cos(theta) sin(theta);-sin(theta) cos(theta)]*(trial.no.mocap.pos(1:2,:)-ipos);
Z=[pos;...
    trial.no.mocap.heading_unwrapped_smooth-theta;...
    trial.no.mocap.longvelocity_smooth;...
    trial.no.mocap.latvelocity_smooth;...
    trial.no.mocap.yawrate_smooth;...
    atan2(trial.no.mocap.yawrate_smooth*.29,trial.no.mocap.longvelocity_smooth)
    trial.no.mocap.longaccel_smooth-trial.no.mocap.latvelocity_smooth.*trial.no.mocap.yawrate_smooth;...
    trial.no.mocap.lataccel_smooth+trial.no.mocap.longvelocity_smooth.*trial.no.mocap.yawrate_smooth];

csvwrite([path,name,'_traj.csv'],[T;Z]');

%write csv file for inputs
U=[intime,...
    trial.no.input.steering(:),...
    trial.no.input.throttle(:)];
csvwrite([path,name,'_input.csv'],U);

%write csv file for imudata (yaw rate, long accel)
is=find(imutime>=starttime,1);
ie=find(imutime<=endtime,1,'last');
meas_imu=[imutime(is:ie)';...
    trial.no.imu.yawrate(is:ie);...
    trial.no.imu.longaccel(is:ie);...
    trial.no.imu.lataccel(is:ie)];
csvwrite([path,name,'_measimu.csv'],meas_imu');
clearvars is ie

%write csv file for encoderdata
is=find(enctime>=starttime,1);
ie=find(enctime<=endtime,1,'last');
vxenc=(trial.no.encoder.left+trial.no.encoder.right)/2;
meas_vx=[enctime(is:ie)';...
        vxenc];
csvwrite([path,name,'_measvx.csv'],meas_vx');
clearvars is ie 

%write csv file for "slam"
is=find(motime>=starttime,1);
ie=find(motime<=endtime,1,'last');
ts=motime(is):1/2:motime(ie);
h=trial.no.mocap.heading_unwrapped_smooth(is:ie)-theta;
pos=pos(1:2,is:ie);
measslam=interp1(motime(is:ie),[pos;h]',ts);
measslam=[ts',measslam];
csvwrite([path,name,'_measslam.csv'],measslam);
clearvars is ie 
%write csv file for recordered stateestimator
% State=[trial.no.state.time';...
%     trial.no.state.pos(1:2,:);...
%     trial.no.state.heading;...
%     trial.no.state.longvelocity;...
%     trial.no.state.wheelangle;...
%     trial.no.state.yawrate;...
%     trial.no.state.longaccel];
% csvwrite([path,name,'_staterecord.csv'],State');