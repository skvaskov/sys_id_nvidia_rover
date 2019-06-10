%
clear all
close all
clc

load('/Users/seanvaskov/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Code_analyze/Processed_Data/steeringdata.mat')
trial=circles_11_29(9);
path=['/Users/seanvaskov/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Code_analyze/State_Estimator/Data/'];
name=['circles'];

%%
time=trial.interp.time;


%write csv file for trajectory
estwheelangle=atan(0.3302*trial.interp.mocap.angular_velocity(3,:)./trial.interp.mocap.velocity(1,:));

Z=[trial.interp.mocap.pos(1:2,:);...
    trial.interp.mocap.orientation(3,:);...
    trial.interp.mocap.angular_velocity(3,:);...
    trial.interp.mocap.velocity(1,:);...
    estwheelangle];
csvwrite([path,name,'_traj.csv'],[time;Z]');

%write csv file for inputs
U=[trial.interp.input.command.steering;...
   trial.interp.input.command.throttle];
U(isnan(U))=0;
csvwrite([path,name,'_input.csv'],[time;U]');

%write csv file for imudata (yaw rate, long accel)
meas_imu=[trial.interp.imu.angular_velocity(3,:);...
    trial.interp.imu.measured_accel(1,:);...
    trial.interp.imu.measured_accel(2,:)];
meas_imu(isnan(meas_imu))=0;
csvwrite([path,name,'_measimu.csv'],[time;meas_imu]');


%write csv file for "slam"

measslam=[trial.interp.mocap.pos(1:2,:);trial.interp.mocap.orientation(3,:)]';
csvwrite([path,name,'_measslam.csv'],[ts',measslam]);

%write csv file for recordered stateestimator
% State=[trial.no.state.time';...
%     trial.no.state.pos(1:2,:);...
%     trial.no.state.heading;...
%     trial.no.state.longvelocity;...
%     trial.no.state.wheelangle;...
%     trial.no.state.yawrate;...
%     trial.no.state.longaccel];
% csvwrite([path,name,'_staterecord.csv'],State');