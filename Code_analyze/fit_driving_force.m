close all
clear all

clc
%use this to find a relationship between pwm to motor command and driving
%force

%% ENTER info
load('throttledata.mat')
training=[data1_11_17,data2_11_17];
clearvars -except training
%load('joystickdata.mat')
%training=[training,data_11_17];
clearvars -except training

degree=2;
throttle_threshold=[-3.0,-0.4];
velocity_threshold=[0.25,3.0];
m=7.780;
l=.3302;
lr=.12;
lf=l-lr;
Iz=.2120;
Pveh=[m,Iz,l,lr];
inputplot=1;

%%
sz=size(training);
untrainedidx=round(rand*(sz(2)-1))+1;
untrained=training(untrainedidx);
training(untrainedidx)=[];


[u0,vx,ax,~,~,~,~,~]=pwm_mocap_points(training,Pveh,velocity_threshold,throttle_threshold,inputplot);



const=get_const_straight(u0,vx,ax,Pveh,degree);


%% check answer 

[u0est,vxest,axest,~,~,~,~,test]=pwm_mocap_points(untrained,Pveh,velocity_threshold,throttle_threshold,inputplot);
while isempty(u0est)
 disp('Plotting trained data! untrained was empty')   
[u0est,vxest,axest,~,~,~,~,test]=pwm_mocap_points(training(round(rand*(sz(2)-2))+1),Pveh,velocity_threshold,throttle_threshold,inputplot);
end
estax=est_ax_straight(u0est,vxest,axest,Pveh,degree,const);
figure
plot(test,axest)
hold on
plot(test,estax)
plot(untrained.interp.time,untrained.interp.mocap.local_accel_smooth)

