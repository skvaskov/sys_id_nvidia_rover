%fit linear tire model by using lateral velocity
clear all
close all
clc

load('yaw_pwm.mat')
threshold=.2;

inputplot=0;
deg='d'; %degree of steering poly fit
slipdegree=1;
sclslip=[1 1];
maxvel=3; %max longitudinal velocity m/s
maxaw=.05; %max angular acceleration
maxax=.05; %max longitudinal acceleration
maxay=.005;%max lateral acceleration
maxsteering=.4; %max wheel angle (Deg)

m=2.759;
Iz=0.0672;
lr=.105;
l=.29;
lf=l-lr;
Iz=0.0672;
P=[m,lr,lf,l];
%accleration parameters
scl=[100,1];
%trial to simulate
trial=turns_200(13);
startv=.5;
%trials with high throttle (>1660): turns_200 8,9,10,11 turns_100
%8,9,10,11,22,23,24,25

data=[circles2D_re(2:end),wheelangle_re(2:end),turns_100_re([2:7,12:21,25:end]),turns_200_re([2:7,12:end])];
%%
sz=size(data);
delta=[];
V=[];
B=[];
w=[];
aw=[];
ay=[];
vy=[];
vx=[];
ax=[];
th=[];
for i=1:sz(2)
    idxs=find(data(i).interp.mocap.speed>=threshold,1);
    idxe=find(data(i).interp.mocap.speed>=threshold,1,'last');
    ax=[ax;data(i).interp.mocap.longaccel_smooth(idxs:idxe)];
    ay=[ay;data(i).interp.mocap.lataccel_smooth(idxs:idxe)];
    aw=[aw;data(i).interp.mocap.yawaccel_smooth(idxs:idxe)];
    w=[w;data(i).interp.mocap.yawrate_smooth(idxs:idxe)];
    V=[V;data(i).interp.mocap.speed_smooth(idxs:idxe)];
    vx=[vx;data(i).interp.mocap.longvelocity_smooth(idxs:idxe)];
    vy=[vy;data(i).interp.mocap.latvelocity_smooth(idxs:idxe)];
    th=[th;data(i).interp.input.throttle(idxs:idxe)];
    delta=[delta;steeringfit(data(i).interp.input.steering(idxs:idxe),deg)];
    B=[B;data(i).interp.mocap.bodyslip_smooth(idxs:idxe)];
end
%% remove points
rm=vx>maxvel;
delta(rm)=[];
V(rm)=[];
B(rm)=[];
vx(rm)=[];
vy(rm)=[];
w(rm)=[];
ay(rm)=[];
aw(rm)=[];
ax(rm)=[];
th(rm)=[];

rm=abs(delta)>maxsteering;
delta(rm)=[];
vx(rm)=[];
vy(rm)=[];
V(rm)=[];
B(rm)=[];
w(rm)=[];
ay(rm)=[];
aw(rm)=[];
ax(rm)=[];
th(rm)=[];

rm=abs(aw)>maxaw;
delta(rm)=[];
V(rm)=[];
B(rm)=[];
vx(rm)=[];
vy(rm)=[];
w(rm)=[];
ay(rm)=[];
aw(rm)=[];
ax(rm)=[];
th(rm)=[];

rm=abs(ay)>maxay;
delta(rm)=[];
V(rm)=[];
B(rm)=[];
vx(rm)=[];
vy(rm)=[];
w(rm)=[];
ay(rm)=[];
aw(rm)=[];
ax(rm)=[];
th(rm)=[];

rm=abs(ax)>maxax;
delta(rm)=[];
V(rm)=[];
B(rm)=[];
vx(rm)=[];
vy(rm)=[];
w(rm)=[];
ay(rm)=[];
aw(rm)=[];
ax(rm)=[];
th(rm)=[];

dtu=unique(delta);
thu=unique(th);

V_m=zeros(length(dtu)*length(thu),1);
B_m=zeros(length(dtu)*length(thu),1);
w_m=zeros(length(dtu)*length(thu),1);
aw_m=zeros(length(dtu)*length(thu),1);
ay_m=zeros(length(dtu)*length(thu),1);
vy_m=zeros(length(dtu)*length(thu),1);
vx_m=zeros(length(dtu)*length(thu),1);
ax_m=zeros(length(dtu)*length(thu),1);
delta_m=zeros(length(dtu)*length(thu),1);
th_m=zeros(length(dtu)*length(thu),1);
c=1;

for i=1:length(dtu)
    for j=1:length(thu)
      th_m(c)=thu(j);
      delta_m(c)=dtu(i);
      idxs=find((delta==dtu(i)).*(th==thu(j)));
      if ~isempty(idxs)
      V_m(c)=mean(V(idxs));
      B_m(c)=mean(B(idxs));
      w_m(c)=mean(w(idxs));
      aw_m(c)=mean(aw(idxs));
      ay_m(c)=mean(ay(idxs));
      vy_m(c)=mean(vy(idxs));
      vx_m(c)=mean(vx(idxs));
      ax_m(c)=mean(ax(idxs));
      c=c+1;
      else
          th_m(c)=[];
          delta_m(c)=[];
          V_m(c)=[];
          B_m(c)=[];
          w_m(c)=[];
          aw_m(c)=[];
          ay_m(c)=[];
          vy_m(c)=[];
          vx_m(c)=[];
          ax_m(c)=[];
      end
    end
end
c=c-1;
%%
% [const,~,~]=get_constslip(delta_m,vx_m,vy_m,w_m,ay_m,aw_m,slipdegree,'large',[m,Iz,l,lr],sclslip);
% [Fyf,Fyr,slipf,slipr]=get_tireforce(delta_m,vx_m,vy_m,w_m,ay_m,aw_m,'large',[m,Iz,l,lr],sclslip);
% subplot(2,1,1)
% plot(abs(slipf),abs(Fyf),'*')
% title('F_{yf}')
% subplot(2,1,2)
% plot(abs(slipr),abs(Fyr),'*')
% title('F_{yr}')
% pf=polyfit(slipf,Fyf,1);
% pr=polyfit(slipr,Fyr,1);
% [est]=combest(delta,vx,P,[pf(1),pr(1)]);
% figure
% plot(delta,B,'*')
% hold on
% plot(delta,est(:,1),'*')
%%
Cc=lsqnonlin(@(C) combdiff(delta,vx,B,w,P,C,scl),[33,150]);
[estC]=combest(delta,vx,P,Cc);
g=9.8065;

Cc_shift=lsqnonlin(@(C) combdiff_shift(delta,vx,B,w,P,C,scl),[33,600,0,0]);
[estC_shift]=combest_shift(delta,vx,[P,Cc_shift(1:2)],Cc_shift(3:4));

kinB=atan(lr*tan(delta)/l);
kinw=sqrt(vx.^2+vy.^2).*cos(kinB)/l.*tan(delta);
figure
subplot(2,2,1)
plot(delta(delta>0),B(delta>0),'b*')
hold on
plot(delta(delta<0),B(delta<0),'r*')
plot(delta,kinB,'g.')
ylabel('Body Slip')
xlabel('Wheel Angle (\delta)')
legend('\delta>0','\delta<0','kinematic')
subplot(2,2,2)
plot(delta(delta>0),w(delta>0),'b*')
hold on
plot(delta(delta<0),w(delta<0),'r*')
plot(delta,kinw,'g.')
legend('\delta>0','\delta<0','kinematic')
ylabel('Yaw Rate')
xlabel('Wheel Angle (\delta)')
subplot(2,2,3)
plot(vx(delta>0),B(delta>0),'b*')
hold on
plot(vx(delta<0),B(delta<0),'r*')
plot(vx,kinB,'g.')
ylabel('Body Slip')
xlabel('Longtidudinal Velocity')
subplot(2,2,4)
plot(vx(delta>0),w(delta>0),'b*')
hold on
plot(vx(delta<0),w(delta<0),'r*')
plot(vx,kinw,'g.')
ylabel('Yaw Rate')
xlabel('Longitudinal Velocity')


%%

%combined est
function [cdiff]=combdiff_shift(delta,vx,B,w,P,C,scl)
%P=[m,lr,lf,l];
%C=[Cf,Cr]
%S=[sf,sr]
s=sign(delta);
delta=delta.*s;
B=B.*s;
w=w.*s;
m=P(1);
lr=P(2);
lf=P(3);
l=P(4);
cf=P(1);
cr=P(2);
sf=C(3);
sr=C(4);
g=9.8065;
kus=m*g*lr/(l*cf)-m*g*lf/(l*cr);
estw=((delta-sr/cr+sf/cf).*vx).*((l+kus*vx.^2/g).^-1);
latvest=(lr-lf*m*vx.^2/(l*cr)).*estw+sr/cr*vx;
Best=latvest.*(vx.^-1);
Bdiff=abs(Best-B);
wdiff=abs(estw-w);
cdiff=scl(1)*Bdiff+scl(2)*wdiff;
end

function [est]=combest_shift(delta,vx,P,S)
%P=[m,lr,lf,l];
%C=[Cf,Cr]
s=sign(delta);
delta=delta.*s;
m=P(1);
lr=P(2);
lf=P(3);
l=P(4);
cf=P(5);
cr=P(6);
sf=S(1);
sr=S(2);
g=9.8065;
kus=m*g*lr/(l*cf)-m*g*lf/(l*cr);
estw=((delta-sr/cr+sf/cf).*vx).*((l+kus*vx.^2/g).^-1);
latvest=(lr-lf*m*vx.^2/(l*cr)).*estw+sr/cr*vx;
estB=latvest.*(vx.^-1);
estw=estw.*s;
estB=estB.*s;
est=[estB,estw];
end

function [cdiff]=combdiff(delta,vx,B,w,P,C,scl)
%P=[m,lr,lf,l];
%C=[Cf,Cr]
%S=[sf,sr]
s=sign(delta);

m=P(1);
lr=P(2);
lf=P(3);
l=P(4);
cf=C(1);
cr=C(2);

g=9.8065;
kus=m*g*lr/(l*cf)-m*g*lf/(l*cr);
estw=((delta).*vx).*((l+kus*vx.^2/g).^-1);
latvest=(lr-lf*m*vx.^2/(l*cr)).*estw;
Best=latvest.*(vx.^-1);
Bdiff=abs(Best-B);
wdiff=abs(estw-w);
cdiff=scl(1)*Bdiff+scl(2)*wdiff;
end

function [est]=combest(delta,vx,P,C)
%P=[m,lr,lf,l];
%C=[Cf,Cr]
s=sign(delta);

m=P(1);
lr=P(2);
lf=P(3);
l=P(4);
cf=C(1);
cr=C(2);
sf=0;
sr=0;
g=9.8065;
kus=m*g*lr/(l*cf)-m*g*lf/(l*cr);
estw=(delta.*vx).*((l+kus*vx.^2/g).^-1);
latvest=(lr-lf*m*vx.^2/(l*cr)).*estw;
estB=latvest.*(vx.^-1);

est=[estB,estw];
end

