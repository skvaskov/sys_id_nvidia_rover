function [ output_args ] = plot_pwmchart(data,idx,threshold,const,const_imu,degree,param,Psteer)
%plot the chart with stuff we care about for pwm points
idxs=find(data(idx).interp.mocap.speed>=threshold,1);
idxe=find(data(idx).interp.mocap.speed>=threshold,1,'last');
pa=data(idx).interp.input.throttle(idxs:idxe);
va=data(idx).interp.mocap.longvelocity_smooth(idxs:idxe);
aa=data(idx).interp.mocap.longaccel(idxs:idxe);
as=data(idx).interp.mocap.longaccel_smooth(idxs:idxe);
pa=pa(:);
va=va(:);
tm=data(idx).interp.time(idxs:idxe);
esta(pa>=1575)=est_axv_turn(pa(pa>=1575),va(pa>=1575),vyas(pa>=1575),was(pa>=1575),da(pa>=1575),const(:,1),degree(1),Pveh,Psteer);
esta(pa<=1465)=est_axv_turn(pa(pa<=1465),va(pa<=1465),vyas(pa<=1465),was(pa<=1465),da(pa<=1465),const(:,2),degree(2),Pveh,Psteer);
esta(logical((pa<1575).*(pa>1465)))=est_axv_turn(pa(logical((pa<1575).*(pa>1465))),va(logical((pa<1575).*(pa>1465))),vyas(logical((pa<1575).*(pa>1465))),was(logical((pa<1575).*(pa>1465))),da(logical((pa<1575).*(pa>1465))),const(:,3),degree(3),Pveh,Psteer);



pi=data(idx).interp.input.throttle(idxs:idxe);
vi=(data(idx).interp.encoder.left_smooth(idxs:idxe)+data(idx).interp.encoder.right_smooth(idxs:idxe))/2*1.3/1.193;
ai=data(idx).interp.imu.longaccel(idxs:idxe);
ais=data(idx).interp.imu.longaccel_smooth(idxs:idxe);
pi=pi(:);
vi=vi(:);
esta_imu(pi>=1575)=est_axv_turn(pi(pi>=1575),vi(pi>=1575),vyis(pi>=1575),wis(pi>=1575),di(pi>=1575),const(:,1),degree(1),Pveh,Psteer);
esta_imu(pi<=1465)=est_axv_turn(pi(pi<=1465),vi(pi<=1465),vyis(pi<=1465),wis(pi<=1465),di(pi<=1465),const(:,2),degree(2),Pveh,Psteer);
esta_imu(logical((pi<1575).*(pi>1465)))=est_axv_turn(pi(logical((pi<1575).*(pi>1465))),vi(logical((pi<1575).*(pi>1465))),vyis(logical((pi<1575).*(pi>1465))),wis(logical((pi<1575).*(pi>1465))),di(logical((pi<1575).*(pi>1465))),const(:,3),degree(3),Pveh,Psteer);
name=data(idx).name;

figure
subplot(2,2,1)
plot(tm,aa,'k')
hold on
plot(tm,as,'r')
plot(tm,esta,'b')
legend('unsmooth','smooth', 'estimated')
ylabel('acceleration')
title([name,': mocap'],'Interpreter','none')
subplot(2,2,2)
plot(tm,ai,'k')
hold on
plot(tm,ais,'r')
plot(tm,esta_imu,'c')
legend('usmooth','smooth', 'estimated')
title([name,': imu/encoder'],'Interpreter','none')
subplot(2,2,3)
plot(tm,pa,'g')
xlabel('time(s)')
title('input')
subplot(2,2,4)
plot(tm,va,'r')
hold on
plot(tm,vi,'r--')
xlabel('time(s)')
legend('mocap','encoder')
title('velocity')



end

