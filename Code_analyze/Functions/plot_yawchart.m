function [estay,estaw,slipf,slipr] = plot_yawchart(data,idx,threshold,const,degree,slipmodel,param)
%plot the chart with stuff we care about for pwm points
idxs=find(data(idx).interp.mocap.speed>=threshold,1);
idxe=find(data(idx).interp.mocap.speed>=threshold,1,'last');
P=data(idx).interp.input.steering(idxs:idxe);
delta=steeringfit(P,1);
vx=data(idx).interp.mocap.longvelocity_smooth(idxs:idxe);
vy=data(idx).interp.mocap.latvelocity_smooth(idxs:idxe);
ay=data(idx).interp.mocap.lataccel(idxs:idxe);
ays=data(idx).interp.mocap.lataccel_smooth(idxs:idxe);
aw=data(idx).interp.mocap.yawaccel(idxs:idxe);
w=data(idx).interp.mocap.yawrate_smooth(idxs:idxe);
aws=data(idx).interp.mocap.yawaccel_smooth(idxs:idxe);
tm=data(idx).interp.time(idxs:idxe);

[estay,estaw,slipf,slipr]=est_constslip(delta,vx,vy,w,const,degree,slipmodel,param);

%name=data(idx).name;
name=[];
figure
subplot(2,2,1)
plot(tm,ay,'k')
hold on
plot(tm,ays,'r')
plot(tm,estay,'b')
legend('unsmooth','smooth', 'estimated')
ylabel('lateral acceleration')
title([name,': mocap'],'Interpreter','none')
subplot(2,2,2)
plot(tm,aw,'k')
hold on
plot(tm,aws,'r')
plot(tm,estaw,'c')
ylabel('angular acceleration')
legend('usmooth','smooth', 'estimated')
title([name,': mocap'],'Interpreter','none')
subplot(2,2,3)
plot(tm,delta,'g')
xlabel('time(s)')
title('steeringangle')
subplot(2,2,4)
plot(tm,slipf,'b')
hold on
plot(tm,slipr,'r')
xlabel('time(s)')
legend('front','rear')
title('slip angle')



end

