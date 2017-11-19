function [start_index] =get_start(accX,time,threshold,thresholdtime,n)
%take accelerationdata and get start point
%threshold is the value where you are certain it is accelerating forward
%threshold time is the time you are certain it will get to the threshold by
%take still data based off of first n points (you are sure it is still for
%these points
time=time(~isnan(accX));
accX=accX(~isnan(accX));
accX_shift=smooth(time,accX-mean(accX(1:n)));
sure=find(accX_shift>=threshold,1);
if isempty(sure)
    start_index=1;
else
    still=find(time>=time(sure)-thresholdtime,1);
    still=max(round(sure/2),still);
    start_index=sure;
    while accX_shift(start_index)>max(accX_shift(1:still))
        start_index=start_index-1;
    end
end
%find the first value where the signal crosses the mean working backwards
%from the threshold


end
