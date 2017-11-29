close all 
clear all
clc

 c=[-32.3444674451266;-107.4936376966535;5.4559263010323;35.5572000100370;0.7595495751754;-48.4489780846895;21.5881438769874];%0.5 ms to 3r
%c=[-28.5473219991892;-92.9261353580101;3.89945961480451;24.1696283921364;-0.273898634774999;-41.3476541719846;15.9439724925004];%0.25 to 3.0 ms
cmd=-0.4:-0.001:-0.6;
ss=zeros(size(cmd));
for i=1:length(cmd)
    u0=cmd(i);
    a0=c(1)+c(2)*u0+c(6)*u0^2;
    a1=c(3)+c(4)*u0+c(7)*u0^2;
    a2=c(5);
    rt=roots([a2,a1,a0]);
    idx=find((rt<3) & (rt>0),1);
    ss(i)=rt(idx);
end
ss_est=[-10.445339156721717,-3.584452482313747];
figure
plot(cmd,ss)
hold on
plot(cmd,polyval(ss_est,cmd))

load('steady_state_speeds.mat')
plot(throttle_in,v_mocap,'b*')