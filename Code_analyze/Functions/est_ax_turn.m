function [ax] = est_ax_straight(u0,u1,vx,w,Pveh,deg,const)
%input throttle input (P), longitudinal velocity (vx), and nx1 constants for the model
%output. acceleration (ax) in nx1 column vecort 

m=Pveh(1);
Iz=Pveh(2);
lr=Pveh(4);
l=Pveh(3);
delta=0.224314009055080*u1-0.008867066788855;
mo=(m*lr^2+Iz)/l^2;

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

   b=A*const;
   ax=b./(m+mo*tan(delta).^2);
end

