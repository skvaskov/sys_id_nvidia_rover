clear all
close all
clc

load('throttledata.mat')
coastthreshold=-0.35;
brakethreshold=0;
data=[decel_11_28,data2_11_17];
sz=size(data);
figure
for i=1:sz(2)
    idxes=(data(i).interp.input.command.throttle<brakethreshold)&(data(i).interp.input.command.throttle>coastthreshold);
    subplot(1,2,1)
    plot(data(i).interp.time(idxes),data(i).interp.mocap.velocity(1,idxes))
    hold on
    idxes=data(i).interp.input.command.throttle>brakethreshold;
    subplot(1,2,2)
    plot(data(i).interp.time(idxes),data(i).interp.mocap.velocity(1,idxes))
    hold on
    
    
end