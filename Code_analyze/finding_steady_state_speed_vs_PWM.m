clear ; clc ; close all
load('steeringdata.mat')
data=circles;

%%
sz=size(data);
% matrices to be filled in by the loop:
throttle_in = nan(sz(2),1) ;
v_mocap = nan(sz(2),1) ;


% for each trial...
for idx = 1:sz(2)
    
    figure(1)
    hold off
    % plot the mocap data and have the user enter start and end times
    plot(data(idx).interp.time,data(idx).interp.mocap.velocity(1,:))
    hold on
    plot(data(idx).interp.time,data(idx).interp.input.command.throttle)
    grid on
    
    tlo = input(['Trial ',num2str(idx),' t start: ']) ;
    thi = input(['Trial ',num2str(idx),' t end: ']) ;
    
    if thi>tlo
    t_idx = (data(idx).interp.time >= tlo) & (data(idx).interp.time <= thi) ;
    v_mocap(idx) = mean(data(idx).interp.mocap.velocity(1,t_idx)) ;
    throttle_in(idx)=mode(data(idx).interp.input.command.throttle(t_idx));
    end
   
end

close all

%% plotting results
[throttle_sort,sort_idx] = sort(throttle_in) ;
l=(~isnan(throttle_in)) & (throttle_in>=-0.525);
pv=polyfit(throttle_in(l),v_mocap(l),1);
figure
plot(throttle_sort,v_mocap(sort_idx),'b*','LineWidth',1.25) ; hold on
plot(throttle_in,polyval(pv,throttle_in),'r')
legend('mocap','fit')
set(gca,'FontSize',15)

xlabel('PWM Command') ; ylabel('Speed [m/s]')