close all
clear all

clc
%use this to find a relationship between pwm to motor command and driving
%force

%% ENTER info
load('throttledata.mat')
training=[data1_11_28];
load('steeringdata.mat')
training=[training,circles_11_29([3,5:end]),turns_11_29];
clearvars -except training


degree=2;
throttle_threshold=[-1.0,-0.4];
velocity_threshold=[0.4,3.0];
m=7.780;
l=.3302;
lr=.12;
lf=l-lr;
Iz=.2120;
Pveh=[m,Iz,l,lr];
inputplot=0;
%% lsq nonline fit for kus
%kus 
%%
sz=size(training);
% untrainedidx=round(rand*(sz(2)-1))+1;
% untrained=training(untrainedidx);
% training(untrainedidx)=[];


[~,~,vx,~,vy,w,delta,~,~]=pwm_mocap_points(training,Pveh,velocity_threshold,throttle_threshold,inputplot);
figure
plot3(vx,delta,w,'*')

grid on
[xdata,ydata,zdata]=prepareSurfaceData(vx,delta,w);
Zfit=fit([xdata,ydata],zdata,'poly23');
hold on
plot3(vx,delta,vx/0.3302.*tan(delta),'*')
plot(Zfit,[xdata ydata],zdata)
ylim([-.66,.66])
xlim([0,2.5])