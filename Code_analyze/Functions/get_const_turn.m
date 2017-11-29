function [const] = get_const_turn(u0,vx,ax,Pveh,deg)
%column vectors of inu0ut, velocity, local acceleration,yaw rate, wheel
%angle, wheel angle dot, u0veh=[m,Iz,l,lr], degree is degree fit of
%polynomial

m=Pveh(1);


b=ax*m;

if deg==0
    A=[ones(size(u0)) vx vx.^2];        
end
if deg==1
     A=[ones(size(u0)) u0 vx vx.^2];    
end

if deg==2
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2];   
end

if deg==3
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2 u0.*vx.^2 u0.^3 vx.^3];   
end
if deg==4
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2 u0.*vx.^2 vx.^3 u0.^3 vx.^2.*u0.^2 vx.*u0.^3 u0.*vx.^3 u0.^4 vx.^4];   
end
const=A\b;
    figure
    plot(b)
    hold on
    plot(A*const)
    title(num2str(const))
end

