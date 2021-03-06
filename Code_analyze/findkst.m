%used this script to look at "S turn" data to determine how quickly the
%steering motor responds to input
clear all
close all
clc
load('steeringdata.mat')
data=sturns;
l=.3302;
pr=[ 0.224314009055080  -0.008867066788855];
t0=1.8;
%%
sz=size(data);

b=[];
A=[];

y0=zeros(1,sz(2));
delta0=zeros(1,sz(2));
ddes=zeros(1,sz(2));
for i=1:sz(2)
%     idxs=find(data(i).interp.mocap.velocity(1,:)>.5,1);
%     idxe=find(data(i).interp.mocap.velocity(1,:)>.5,1,'last');
    idxs=find(data(i).interp.time>t0,1);
    idxe=find(data(i).interp.time>(t0+1.5),1);
    rg=idxs:idxe;
    estdelta=atan(l*data(i).interp.mocap.angular_velocity(3,rg)./data(i).interp.mocap.velocity(1,rg));
     ddes=polyval(pr,data(i).interp.input.command.steering(rg));
     estdeltadt=get_dt(estdelta,data(i).interp.time(rg));
    b=[b;estdeltadt'];
    A=[A;(ddes-estdelta)'];
    y0(i)=data(i).interp.mocap.angular_velocity(3,idxs);
    delta0=0;
   
end

k=A\b

for i=1:sz(2)
    estdelta=atan(l*data(i).interp.mocap.angular_velocity(3,:)./data(i).interp.mocap.velocity(1,:));
    ddes=polyval(pr,data(i).interp.input.command.steering);
    idxs=find(data(i).interp.time>t0,1);
    idxe=find(data(i).interp.time>(t0+1.5),1);
    tspan=data(i).interp.time(idxs:idxe);
    y0= estdelta(idxs);
    [~,y]=ode45(@(t,y)k*(zoh(tspan,ddes(idxs:idxe)',t)-y),tspan,y0);
    
    figure(1)
    hold on
    plot(tspan-0.15,y','b','LineWidth',2)
    plot(data(i).interp.time,estdelta,'k.')
    plot(data(i).interp.time,ddes,'r')
end
