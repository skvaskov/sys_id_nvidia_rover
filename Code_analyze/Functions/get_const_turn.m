function [const] = get_const_turn(u0,u1,vx,ax,w,Pveh,deg)
%column vectors of inu0ut, velocity, local acceleration,yaw rate, wheel
%angle, wheel angle dot, u0veh=[m,Iz,l,lr], degree is degree fit of
%polynomial

m=Pveh(1);
Iz=Pveh(2);
lr=Pveh(4);
l=Pveh(3);
delta=0.224314009055080*u1-0.008867066788855;
mo=(m*lr^2+Iz)/l^2;


b=(ax.*(m+mo*tan(delta).^2));
mx=abs(max(b));
if deg==0
    A=[ones(size(u0)) vx vx.^2  w.^2];        
end
if deg==1
     A=[ones(size(u0)) u0 vx vx.^2  w.^2];    
end

if deg==2
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2  w.^2];   
end

if deg==3
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2 u0.*vx.^2 u0.^3 vx.^3  w.^2];   
end
if deg==4
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2 u0.*vx.^2 vx.^3 u0.^3 vx.^2.*u0.^2 vx.*u0.^3 u0.*vx.^3 u0.^4 vx.^4 w.^2];   
end
const=A\(b/mx);
const=const*mx;
    figure
    plot(b)
    hold on
    plot(A*const)
    title(num2str(const))
end

