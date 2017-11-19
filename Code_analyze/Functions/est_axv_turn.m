function [ax] = est_axv_turn(P,vx,vy,w,delta,const,deg,Param,Pst)
%input throttle input (P), longitudinal velocity (vx), and nx1 constants for the model
%output. acceleration (ax) in nx1 column vecort 
m=Param(1);
Iz=Param(2);
l=Param(3);
lr=Param(4);
cf=Pst(1);
sh=Pst(2);
sv=Pst(3);

% slipf=zeros(size(vx));
% for i=1:length(slipf)
% slipf(i)=(vx(i)*sin(delta(i))-(vy(i)+lf*w(i))*cos(delta(i)))/(vx(i)*cos(delta(i))+(vy(i)+lf*w(i))*sin(delta(i)));
% end
% Fyf=cf*(slipf+sh.*sign(delta))+sv;
mo=(m*lr^2+Iz)/l^2;
if deg==0
    A=[ones(size(P)) vx vx.^2];
    b=A*const;
end
if deg==1
    A=[ones(size(P)) P vx P.*vx vx.^2];
    b=A*const;
end
if deg==2
    A=[ones(size(P)) P vx P.*vx vx.^2 P.^2 vx.*P.^2];
    b=A*const;
end
if deg==3
    A=[ones(size(P)) P vx P.*vx vx.^2 P.^2 vx.*P.^2 P.*vx.^2 P.^3 vx.^3];
    b=A*const;
end
if deg==4
    A=[ones(size(P)) P vx P.*vx vx.^2 P.^2 vx.*P.^2 P.*vx.^2 vx.^3 P.^3 vx.^2.*P.^2 vx.*P.^3 P.*vx.^3 P.^4 vx.^4];
    b=A*const;
end
ax=b.*((m+mo*tan(delta).^2).^-1);
end

