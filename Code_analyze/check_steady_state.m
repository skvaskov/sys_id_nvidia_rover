
clear all
clc
l=0.3302;

const=[-26.8598346219134;-87.6546485543967;3.14128477268640;35.5572000100370 ; 0.7595495751754 ;-48.4489780846895 ;15.3781821037200;-0.474095491113532];
cmd=-0.4:-0.001:-0.6;
cmd_u1=[2,0,-2];
ss=zeros(size(cmd));
figure
hold on
for u1=cmd_u1
    delta=0.224314009055080*u1-0.008867066788855;
for i=1:length(cmd)
    
    u0=cmd(i);
    a0=const(1)+const(2)*u0+const(6)*u0^2;
    a1=const(3)+const(4)*u0+const(7)*u0^2;
    a2=const(5)+const(8)/l^2*tan(delta)^2;
    rt=roots([a2,a1,a0]);
    idx=find((rt<3) & (rt>0),1);
    ss(i)=rt(idx);
end
    plot(cmd,ss)
end
ss_est=[-10.445339156721717,-3.584452482313747];


hold on
%plot(cmd,polyval(ss_est,cmd))

load('steady_state_speeds.mat')
plot(throttle_in,v_mocap,'b*')
load('steady_circle.mat')
plot(throttle([3,5:end]),vx([3,5:end]),'r*')