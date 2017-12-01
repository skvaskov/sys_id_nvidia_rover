
clear all
clc
l=0.3302;
const=[-12.5810995587748;-33.0170773577599;4.33920832891501;20.3041178298046;0.156420898500981;4.20678380627274;10.2828808092518;-0.610920415224012];
%const=[-25.1220434879362;-81.7158714056317;2.08170809764619;21.1235710569551;0.237666708745836;-31.5571659476673;12.6153618800570;-0.610920415224012]
%const=[ -2.45016608416963; 3.27793345308585; -3.25849624111027; -10.4089771121760; -0.801029663670134; 28.8049924127278; -10.0302704026886;-0.567741817043089];
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

%check otherway
vx_des=0.7;
w=1.5;
a0=const(1)+const(3)*vx_des+const(5)*vx_des^2+const(8)*w^2;
a1=const(2)+const(4)*vx_des;
a2=const(6)+const(7)*vx_des;
roots([a2,a1,a0])
(-a1-sqrt(a1^2-4*a2*a0))/(2*a2)