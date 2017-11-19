function [Fyf,Fyr,slipf,slipr] = est_kinematic_forces(delta,vx,vy,w,ay,aw,slipmodel,param)
%Estimate of steady state tire forces based of kinematic model
%   Detailed explanation goes here
%P=[m,Iz,l,lr,af,ar]
l=param(3);
lr=param(4);
lf=l-lr;
Iz=param(2);
m=param(1);
m0=(Iz+m*lr^2)/l^2;
slipf=zeros(size(delta));
slipr=zeros(size(slipf));
Fyf=zeros(size(slipf));
Fyr=zeros(size(slipr));
for i=1:length(slipf)
    if slipmodel=='small'
    slipf(i)=delta(i)-(vy(i)+lf*w(i))/vx(i);
    elseif slipmodel=='large'
    slipf(i)=(vx(i)*sin(delta(i))-(vy(i)+lf*w(i))*cos(delta(i)))/(vx(i)*cos(delta(i))+(vy(i)+lf*w(i))*sin(delta(i)));
    elseif slipmodel=='fulll'
    af=param(5);
    slipf(i)=(vx(i)*sin(delta(i))-(vy(i)+lf*w(i))*cos(delta(i))-af*w(i))/(vx(i)*cos(delta(i))+(vy(i)+lf*w(i))*sin(delta(i)));
    end
    if slipmodel=='fulll'
     ar=param(6);
    slipr(i)=((lr-ar)*w(i)-vy(i))/vx(i);
    else
    slipr(i)=(lr*w(i)-vy(i))/vx(i);
    end
    
    Fyf(i)=1/cos(delta(i))*(m*lr/l^2*vx(i)^2*tan(delta(i)));
    Fyr(i)=m/l*tan(delta(i))*(1-lr/l)*vx(i)^2;
end




end

