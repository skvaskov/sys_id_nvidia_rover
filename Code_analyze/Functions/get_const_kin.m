function [const] = get_const_kin(u0,vx,ax,delta,delta_dot,Pveh,deg)
%column vectors of inu0ut, velocity, local acceleration,yaw rate, wheel
%angle, wheel angle dot, u0veh=[m,Iz,l,lr], degree is degree fit of
%polynomial

m=Pveh(1);
Iz=Pveh(2);
l=Pveh(3);
lr=Pveh(4);

mo=(m*lr^2+Iz)/l^2;

b=ax.*(m+mo*tan(delta).^2)+mo*tan(delta)./(cos(delta).^2).*delta_dot.*vx;

if deg==0
    A=[ones(size(u0)) vx vx.^2];
        const=A\b;
end
if deg==1
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2];
    const=A\b;
end

if deg==2
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2];
    const=A\b;
end

if deg==3
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2 u0.*vx.^2 u0.^3 vx.^3];
    const=A\b;
end
if deg==4
    A=[ones(size(u0)) u0 vx u0.*vx vx.^2 u0.^2 vx.*u0.^2 u0.*vx.^2 vx.^3 u0.^3 vx.^2.*u0.^2 vx.*u0.^3 u0.*vx.^3 u0.^4 vx.^4];
    const=A\b;
end
    figure
    plot(b)
    hold on
    plot(A*const)
    title(num2str(const))
end

