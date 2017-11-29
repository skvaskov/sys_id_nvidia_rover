clear ; clc ; close all
load('steeringdata.mat')
data=[circles];

%%
sz=size(data);
% matrices to be filled in by the loop:
throttle = nan(sz(2),1) ;
vx = nan(sz(2),1) ;
vy = nan(sz(2),1);
w = nan(sz(2),1);
delta= nan(sz(2),1);
R= nan(sz(2),1);


% for each trial...
for idx = 1:sz(2)
    
    figure(1)
    hold off
    % plot the mocap data and have the user enter start and end times
    plot(data(idx).interp.time,data(idx).interp.mocap.velocity(1,:),'b')
    hold on
    plot(data(idx).interp.time,data(idx).interp.input.command.throttle,'b')
    plot(data(idx).interp.time,data(idx).interp.mocap.angular_velocity(3,:),'r')
    hold on
    plot(data(idx).interp.time,data(idx).interp.input.command.steering,'r')
    grid on
    
    tlo = input(['Trial ',num2str(idx),' t start: ']) ;
    thi = input(['Trial ',num2str(idx),' t end: ']) ;
    
    if thi>tlo
    t_idx = (data(idx).interp.time >= tlo) & (data(idx).interp.time <= thi) ;
    vx(idx) = mean(data(idx).interp.mocap.velocity(1,t_idx)) ;
    vy(idx) = mean(data(idx).interp.mocap.velocity(2,t_idx));
    w(idx)  = mean(data(idx).interp.mocap.angular_velocity(3,t_idx));
    delta(idx)=0.224314009055080*mode(data(idx).interp.input.command.steering(t_idx))-0.008867066788855;
    throttle(idx)=mode(data(idx).interp.input.command.throttle(t_idx));
    [~,R(idx)]=get_circle(data(idx).interp.mocap.pos(1,t_idx)',data(idx).interp.mocap.pos(2,t_idx)');
    end
   
end

close all

