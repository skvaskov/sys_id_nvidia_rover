function [ax] = est_ax_straight(u0,vx,ax,Pveh,deg,const)
%input throttle input (P), longitudinal velocity (vx), and nx1 constants for the model
%output. acceleration (ax) in nx1 column vecort 
m=Pveh(1);

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
   b=A*const;
   ax=b/m;
end

