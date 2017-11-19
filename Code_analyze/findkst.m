%used this script to look at "S turn" data to determine how quickly the
%steering motor responds to input
clear all

clc
load('reprocessed_circles_5_31.mat')
l=.29;
sz=size(processeddata);
tsp=100;
for i=1:sz(2)
    sw=find(diff(processeddata(i).input.steering));
    sw=sw(sw>10);
    sw=sw(1);
    processeddata(i).time=processeddata(i).time-processeddata(i).time(sw);
    processeddata(i).estdelta=atan2(processeddata(i).mocap.yawrate_smooth*l,processeddata(i).mocap.longvelocity_smooth.*cos(processeddata(i).mocap.bodyslip_smooth));
    processeddata(i).estdelta=smooth(processeddata(i).time,processeddata(i).estdelta);
    processeddata(i).estdeltadt=get_dt(processeddata(i).estdelta, processeddata(i).time);
    processeddata(i).estdeltadt=smooth(processeddata(i).time,processeddata(i).estdeltadt);
    is=find(processeddata(i).time>-1.5,1);
    ie=find(processeddata(i).time>-.5,1);
    processeddata(i).ddes(processeddata(i).time<0)=mean(processeddata(i).estdelta(is:ie));
    is=find(processeddata(i).time>2,1);
    ie=find(processeddata(i).time>3,1);
    processeddata(i).ddes(processeddata(i).time>=0)=mean(processeddata(i).estdelta(is:ie));
    processeddata(i).ddes=processeddata(i).ddes';
    figure(1)
    subplot(2,1,1)
    hold on
    plot(processeddata(i).time,processeddata(i).estdelta,'b')
    plot(processeddata(i).time,processeddata(i).ddes,'r')
    subplot(2,1,2)
    hold on
    plot(processeddata(i).time,processeddata(i).estdeltadt,'b')
end

%%
b=[];
A=[];

y0=zeros(1,sz(2));
delta0=zeros(1,sz(2));
ddes=zeros(1,sz(2));
for i=1:sz(2)
    idxs=find(processeddata(i).time>=0,1);
    idxe=idxs+tsp;
    rg=idxs:idxe;
    b=[b;processeddata(i).estdeltadt(rg)];
    A=[A;(processeddata(i).ddes(rg)-processeddata(i).estdelta(rg))];
    y0(i)=processeddata(i).mocap.yawrate_smooth(idxs);
    delta0=processeddata(i).estdelta(idxs);
    ddes=processeddata(i).ddes(idxe);
end

k=A\b

for i=1:sz(2)
    figure(1)
    subplot(2,1,2)
    hold on
    plot(processeddata(i).time,k*(processeddata(i).ddes-processeddata(i).estdelta),'k')
end
%%
Z0=[mean(y0),mean(delta0)];
r=round(rand*sz(2));
for r=1:sz(2)
Vx=processeddata(r).mocap.longvelocity_smooth;
B=processeddata(r).mocap.yawrate_smooth;
Ddes=processeddata(r).ddes;
it=processeddata(r).time;

P=[l,k];
[t,z]=ode45(@(t,Z) kinsim(t,Z,Vx,B,Ddes,P,it),[0,mean(diff(processeddata(r).time))*tsp],Z0);
figure
subplot(2,1,1)
plot(processeddata(r).time,processeddata(r).estdelta)
hold on
plot(t,z(:,2),'k')
end
function [dZdt]=kinsim(t,Z,Vx,B,Ddes,Param,it)
    l=Param(1);
    k=Param(2);
    vx=interp1(it,Vx,t);
    b=interp1(it,B,t);
    ddes=interp1(it,Ddes,t);
    
    dZdt=[vx/l*cos(b)*tan(Z(2));...
            k*(ddes-Z(2))];
        
end