clear all
close all
clc

load('throttledata.mat')
coastthreshold=-0.35;
brakethreshold=0;
throttle=-1.0;
data=[data2_11_17,data1_11_17];
sz=size(data);
decel=[];
j=1;
figure
for i=1:sz(2)
    idxes=(data(i).interp.input.command.throttle<=throttle);
    subplot(1,2,1)
    plot(data(i).interp.time(idxes),data(i).interp.mocap.velocity(1,idxes))
    hold on
    idxes=data(i).interp.input.command.throttle>brakethreshold;
    subplot(1,2,2)
    if any(data(i).interp.mocap.velocity(1,idxes)>2.0)
        disp(num2str(i))
        decel{j}=[data(i).interp.time(idxes);data(i).interp.mocap.velocity(1,idxes)];
        j=j+1;
    end
    plot(data(i).interp.time(idxes),data(i).interp.mocap.velocity(1,idxes))
    hold on 
end

for j=[1,2,3]
    dist(j)=0;
    data=decel{j};
    sz=size(data);
    for i=1:sz(2)-1
    dist(j)=dist(j)+(data(2,i)*(data(1,i+1)-data(1,i))+data(2,i+1)*(data(1,i+1)-data(1,i)))/2;
    end
end
