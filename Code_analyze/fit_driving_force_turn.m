close all
clear all

clc
%use this to find a relationship between pwm to motor command and driving
%force

%% ENTER info
load('throttledata.mat')
%training=[data1_11_17,data2_11_17,data1_11_28,decel_11_28];
training=[decel_11_28];
load('steeringdata.mat')
%training=[training,circles_11_29([3,5:end]),turns_11_29,decel_11_28];
training=[training,decel_11_28];
clearvars -except training


degree=1;
throttle_threshold=[0.0,3.0];
velocity_threshold=[0.0,3.0];
m=7.780;
l=.3302;
lr=.12;
lf=l-lr;
Iz=.2120;
Pveh=[m,Iz,l,lr];
inputplot=0;

%%
sz=size(training);
untrainedidx=round(rand*(sz(2)-1))+1;
untrained=training(untrainedidx);
training(untrainedidx)=[];


[u0,u1,vx,ax,~,w,~,~,~]=pwm_mocap_points(training,Pveh,velocity_threshold,throttle_threshold,inputplot);



const=get_const_turn(u0,u1,vx,ax,w,Pveh,degree);


%% check answer 

[u0est,u1est,vxest,axest,~,west,~,~,test]=pwm_mocap_points(untrained,Pveh,velocity_threshold,throttle_threshold,inputplot);
while isempty(u0est)
 disp('Plotting trained data! untrained was empty')   
[u0est,u1est,vxest,axest,~,west,~,~,test]=pwm_mocap_points(training(round(rand*(sz(2)-2))+1),Pveh,velocity_threshold,throttle_threshold,inputplot);
end
estax=est_ax_turn(u0est,u1est,vxest,west,Pveh,degree,const);
figure
plot(test,axest)
hold on
plot(test,estax)
plot(untrained.interp.time,untrained.interp.mocap.local_accel(1,:))

