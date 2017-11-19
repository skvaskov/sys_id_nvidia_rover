%use this to find the map from pwm to wheel angle

clear all
close all
clc

load('steeringdata.mat')
threshold=.25;
inputplot=0;
deg=1; %degree of steering poly fit
maxvel=1; %max longitudinal velocity m/s
maxaw=.05; %max angular acceleration
maxay=.005;%max lateral acceleration
maxsteering=300; %max wheel angle (Deg)
m=7.78;
lr=.12;
l=.3302;
lf=l-lr;

data=[wheelangle_trials(3:end)];
sz=size(data);
steer=[];
V=[];
B=[];
w=[];
aw=[];
ay=[];
vy=[];
vx=[];
RP=zeros(2,sz(2));
for i=1:sz(2)
    idxs=find(data(i).interp.mocap.velocity(1,:)>=threshold,1);
    idxe=find(data(i).interp.mocap.velocity(1,:)>=threshold,1,'last');
  
    tb=data(i).interp.time(end)-.25;
    ts=tb;
   
    ire=find(data(i).interp.time<=tb,1,'last');
    [~,RP(2,i)]=get_circle(data(i).interp.mocap.pos(1,:)',data(i).interp.mocap.pos(2,:)');
    RP(1,i)=mode(data(i).interp.input.command.steering);
    
    w=[w,data(i).interp.mocap.angular_velocity(3,idxs:idxe)];

    vx=[vx,data(i).interp.mocap.velocity(1,idxs:idxe)];
   
    steer=[steer,data(i).interp.input.command.steering(idxs:idxe)];
 
end


%%

estdelta=atan2(l*w,vx);
sortdata=sortrows([steer,estdelta]);
steermean=unique(sortdata(:,1));
anglemean=zeros(length(steermean),1);
for i=1:length(steermean)
idx=sortdata(:,1)==steermean(i);
anglemean(i)=mean(sortdata(idx,2));
end
p=polyfit(steer,estdelta,deg);
estdeltaR=sign(RP(1,:))./RP(2,:)*l;
pR=polyfit(RP(1,:),estdeltaR,1);
% idr=RP(1,:)<0;
% idl=RP(1,:)>0;
%pr=polyfit(RP(1,idr),estdeltaR(idr),1);
%pl=polyfit(RP(1,idl),estdeltaR(idl),1);
% zr=-pr(1)/pr(2);
% zl=-pl(1)/pl(2);
%  inz=[-3:.1:zr,zr:.1:zl,zl:.1:3];
%  deadz=[polyval(pr,-3:.1:zr),zeros(size(zr:.1:zl)),polyval(pl,zl:.1:3)];
in=-3:0.1:3;
figure
plot(steer,estdelta,'*')
hold on
plot(RP(1,:),estdeltaR,'*')
hold on
plot(in,polyval(p,in))
plot(in,polyval(pR,in))
xlabel('input')
ylabel('wheel angle')
legend('\omega, v_x data','Radius data','\omega, v_x fit','Radius fit')


