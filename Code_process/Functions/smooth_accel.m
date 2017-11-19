function [accelS] = smooth_accel(accel,acceltime,steering,throttle,throttletime,threshold,buffer,factor)
%input acceleration points 3xn vector with coressponding time points 1xn
%output smooth 3xxn acceleration vector smooth with 'sgolay' function
 accelS=[smooth(acceltime,accel(1,:))';...
         smooth(acceltime,accel(2,:))';...
         smooth(acceltime,accel(3,:))'];
%find change in throttle (assume it is nonzero at first timestep)    

changet=diff(throttle);
idxs=find(abs(changet)>threshold);
changetimes=throttletime(idxs);
changes=diff(steering);
idxss=find(changes>threshold);
changetimes=[changetimes;throttletime(idxss)];
changetimes=unique(changetimes);
%add buffer around timesteps where its changing to avoid smoothing there
changetimesbuff=changetimes+buffer;
changetimes=[changetimes;changetimesbuff];
changetimes=sort(changetimes);

diffchangetimessup=diff(changetimes);
idxs=find(diffchangetimessup>buffer);
%if there are time instacnes larger than the buffer apply more smoothing
if isempty(idxs)==0
for i=1:length(idxs)
    dt=changetimes(idxs(i)+1)-changetimes(idxs(i));
    istart=find(acceltime>=changetimes(idxs(i)),1);
    iend=find(acceltime<=changetimes(idxs(i)+1),1,'last');
    if istart~=iend
    accelS(:,istart:iend)=[smooth(acceltime(istart:iend),accel(1,istart:iend),factor*dt)';...
                            smooth(acceltime(istart:iend),accel(2,istart:iend),factor*dt)';...
                            smooth(acceltime(istart:iend),accel(3,istart:iend),factor*dt)'];
    end
end



end
end

