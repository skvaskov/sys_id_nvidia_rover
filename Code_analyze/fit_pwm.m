
clear all

clc
%use this to find a relationship between pwm to motor command and driving
%force
Pm=[1480*ones(1000,1);1500*ones(1000,1);1520*ones(1000,1);1540*ones(1000,1);1560*ones(1000,1);1580*ones(1000,1)];

vx=zeros(size(Pm));
ax=zeros(size(vx));
vy=zeros(size(vx));
w=zeros(size(vx));
wi=w;
vyi=vy;
Pst=rand(size(vx))*pi/4-pi/8;
Psti=Pst;
Pi=Pm;
aximu=ax;
vxenc=vx;
%% ENTER info
threshold=.2;
load('motor_pwm.mat')
load('yaw_pwm.mat')
degree=2;
maxthrottle=1700;
dispdata=circles2D;
disptrial=[3];
m=2.759;
l=.29;
lr=.105;
lf=l-lr;
Iz=.0672;
Pveh=[m,Iz,l,lr];
cf=44.74;
inputplot=1;
Psteer=[37.83,0,0];
training=[datarem(3:65),datarem2(3:69),datarem3([3:28,34:47]),data1700(1:12),dataaccel(1:1:9)];
trainingturn=[wheelangle_re(2:end),turns_200_re([2:7,12:end]),turns_100_re([2:6,12:21,25:end]),circles2D_re(2:end)];

sz=size(training);
szt=size(trainingturn);
untrained=[data1700(12:end),dataslow];
enum=1;
%%
[Pm,vx,ax,vy,w,Pst]=pwm_mocap_points_turn(training,1:sz(2),threshold,inputplot,Pm,vx,ax,vy,w,Pst);
[Pm,vx,ax,vy,w,Pst]=pwm_mocap_points_turn(trainingturn,1:szt(2),threshold,inputplot,Pm,vx,ax,vy,w,Pst);
[Pm,vx,ax,vy,w,Pst]=pwm_mocap_points(datacruisemocap,1:25,threshold,inputplot,Pm,vx,ax,vy,w,Pst);
%remove things greater than max throttle
rm=Pm>maxthrottle;
Pm(rm)=[];
vx(rm)=[];
ax(rm)=[];
vy(rm)=[];
w(rm)=[];
Pst(rm)=[];


[Pi,vxenc,aximu,vyi,wi,Psti]=pwm_imu_points_turn(training,1:sz(2),threshold,0,Pi,vxenc,aximu,vyi,wi,Psti);
[Pi,vxenc,aximu,vyi,wi,Psti]=pwm_imu_points_turn(trainingturn,1:szt(2),threshold,0,Pi,vxenc,aximu,vyi,wi,Psti);
[Pi,vxenc,aximu,vyi,wi,Psti]=pwm_imu_points(datacruiseimu,1:25,threshold,0,Pi,vxenc,aximu,vyi,wi,Psti);
%remove things greater than max throttle
rmi=Pm>maxthrottle;
Pi(rmi)=[];
vxenc(rmi)=[];
aximu(rmi)=[];
vyi(rm)=[];
wi(rm)=[];
Psti(rm)=[];

const=get_constv_turn(Pm,vx,ax,vy,w,steeringfit(Pst,'d'),degree,Pveh,Psteer);
const_imu=get_constv_turn(Pi,vxenc,aximu,vyi,wi,steeringfit(Psti,'d'),degree,Pveh,Psteer);


%% check answer 
if ~isempty(disptrial)
for i=length(disptrial)
idxs=find(dispdata(disptrial(i)).interp.mocap.speed>=threshold,1);
idxe=find(dispdata(disptrial(i)).interp.mocap.speed>=threshold,1,'last');
pa=dispdata(disptrial(i)).interp.input.throttle(idxs:idxe);
va=dispdata(disptrial(i)).interp.mocap.longvelocity_smooth(idxs:idxe);
vyas=dispdata(disptrial(i)).interp.mocap.latvelocity_smooth(idxs:idxe);
was=dispdata(disptrial(i)).interp.mocap.yawrate_smooth(idxs:idxe);
aa=dispdata(disptrial(i)).interp.mocap.longaccel(idxs:idxe);
as=dispdata(disptrial(i)).interp.mocap.longaccel_smooth(idxs:idxe);
ps=dispdata(disptrial(i)).interp.input.steering(idxs:idxe);
da=steeringfit(ps,'d');
pa=pa(:);
va=va(:);
tm=dispdata(disptrial(i)).interp.time(idxs:idxe);
esta=est_axv_turn(pa,va,vyas,was,da,const,degree,Pveh,Psteer);


pi=dispdata(disptrial(i)).interp.input.throttle(idxs:idxe);
vi=(dispdata(disptrial(i)).interp.encoder.left_smooth(idxs:idxe)+dispdata(disptrial(i)).interp.encoder.right_smooth(idxs:idxe))/2*1.3/1.193;
ai=dispdata(disptrial(i)).interp.imu.longaccel(idxs:idxe);
ais=dispdata(disptrial(i)).interp.imu.longaccel_smooth(idxs:idxe);
vyis=dispdata(disptrial(i)).interp.mocap.latvelocity_smooth(idxs:idxe);
wis=dispdata(disptrial(i)).interp.imu.yawrate_smooth(idxs:idxe);
psi=dispdata(disptrial(i)).interp.input.steering(idxs:idxe);
dai=steeringfit(psi,'d');
pi=pi(:);
vi=vi(:);
esta_imu=est_axv_turn(pi,vi,vyis,wis,dai,const_imu,degree,Pveh,Psteer);


figure
subplot(1,2,1)
plot(tm,aa,'k')
hold on
plot(tm,as,'r')
plot(tm,esta,'b')
legend('unsmooth','smooth', 'estimated')
ylabel('acceleration')
xlabel('time(s)')
title('mocap')
subplot(1,2,2)
plot(tm,ai,'k')
hold on
plot(tm,ais,'r')
plot(tm,esta_imu,'b')
xlabel('time(s)')
legend('usmooth','smooth', 'estimated')
title('imu')

end
end
%% find l2 norm of untrained trials and rank
sz=size(untrained);
mocaperror=zeros(2,sz(2));
imuerror=zeros(2,sz(2));
for i=1:sz(2)
    [P_out,vx_out,ax_out,vy_out,w_out,Pst_out]=pwm_mocap_points(untrained,i,threshold,0,[],[],[],[],[],[]);
    [Pi_out,vxi_out,axi_out,vyi_out,wi_out,Psti_out]=pwm_imu_points(untrained,i,threshold,0,[],[],[],[],[],[]);
    esta=est_axv_turn(P_out,vx_out,vy_out,w_out,steeringfit(Pst_out,'d'),const,degree,Pveh,Psteer);
    esta_imu=est_axv_turn(Pi_out,vxi_out,vyi_out,wi_out,steeringfit(Psti_out,'d'),const_imu,degree,Pveh,Psteer);
    mocaperror(1,i)=i;
    mocaperror(2,i)=sum(sqrt((esta-ax_out).^2));
    imuerror(1,i)=i;
    imuerror(2,i)=sum(sqrt((esta_imu-axi_out).^2));
end
mocaperror_sort=sortrows(mocaperror',-2);
imuerror_sort=sortrows(imuerror',-2);
disp(['Top ',num2str(enum),' mocap error: '])
for i=1:enum
    disp([untrained(mocaperror_sort(i,1)).name,'. Error: ',num2str(mocaperror_sort(i,2))])
    
end
disp(['Top ',num2str(enum),' imu error: '])
for i=1:enum
    disp([untrained(imuerror_sort(i,1)).name,'. Error: ',num2str(imuerror_sort(i,2))])
end
un=unique([mocaperror_sort(1:enum,1);imuerror_sort(1:enum,1)]);
for i=1:length(un)
    plot_pwmchart(untrained,un(i,1),threshold,const,const_imu,degree,Pveh,Psteer)
end
%%
% simulate

% % %%
% ts=[1,4,.3,10];
% ys=[1660,1620,1400,1600];
% tssum=cumsum(ts);
% [T,Y]=ode45(@(t,y) motordyn(t,y,ys(1),const),[0,tssum(1)],0);
% for i=2:length(ts)
% [t,y]=ode45(@(t,y) motordyn(t,y,ys(i),const),[tssum(i-1),tssum(i)],Y(end));
% T=[T;t];
% Y=[Y;y];
% end
% figure(3)
% plot(T,Y)
% function dydt=motordyn(t,y,u,const)
%     dydt=[1 u y u*y y^2 u^2 y*u^2]*const;
% end
% 
