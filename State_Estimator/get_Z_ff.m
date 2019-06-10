function Z_ff=get_Z_ff(t,data)
        psi0=minimize_angle(data.initial_pose.orientation-data.road_heading);
        b=data.b;
        Th=data.Th;
        V=data.V;
        lr=0.12;
        Z_ff=zeros(6,1);
        Z_ff(3)=-(Th*b+psi0)/Th^2*t^2+b*t+psi0+data.road_heading;
        Z_ff(6)=-2*(Th*b+psi0)/Th^2*t+b;
        Z_ff(5)=lr*Z_ff(5);
        Z_ff(4)=V;
        dx=(t^8*(2*lr*Th^4*b^4 + 8*lr*Th^3*b^3*psi0 + 12*lr*Th^2*b^2*psi0^2 + 8*lr*Th*b*psi0^3 + 2*lr*psi0^4))/(48*Th^8) - (t^6*(- 9*lr*Th^6*b^4 - 12*lr*Th^5*b^3*psi0 + 9*lr*Th^4*b^2*psi0^2 + 18*lr*Th^3*b*psi0^3 + 6*lr*Th^2*psi0^4))/(36*Th^8) - (t^2*(- 3*lr*Th^8*b^2*psi0^2 + 6*lr*Th^8*b^2 + 6*V*Th^8*b*psi0 + 2*lr*Th^7*b*psi0^3 - 12*lr*Th^7*b*psi0 + 2*lr*Th^6*psi0^4 - 12*lr*Th^6*psi0^2))/(12*Th^8) + (t*(b*lr*Th^8*psi0^3 - 3*V*Th^8*psi0^2 - 6*b*lr*Th^8*psi0 + 6*V*Th^8))/(6*Th^8) + (t^4*(lr*Th^8*b^4 - 12*lr*Th^7*b^3*psi0 + 6*V*Th^7*b^2 - 6*lr*Th^6*b^2*psi0^2 - 12*lr*Th^6*b^2 + 6*V*Th^6*b*psi0 + 12*lr*Th^5*b*psi0^3 - 24*lr*Th^5*b*psi0 + 6*lr*Th^4*psi0^4 - 12*lr*Th^4*psi0^2))/(24*Th^8) - (t^5*(5*lr*Th^7*b^4 - 10*lr*Th^6*b^3*psi0 + 3*V*Th^6*b^2 - 30*lr*Th^5*b^2*psi0^2 + 6*V*Th^5*b*psi0 - 15*lr*Th^4*b*psi0^3 + 3*V*Th^4*psi0^2))/(30*Th^8) + (t^3*(3*lr*Th^8*b^3*psi0 - 3*V*Th^8*b^2 - 9*lr*Th^7*b^2*psi0^2 + 18*lr*Th^7*b^2 + 6*V*Th^7*b*psi0 - 9*lr*Th^6*b*psi0^3 + 18*lr*Th^6*b*psi0 + 6*V*Th^6*psi0^2))/(18*Th^8) - (t^7*(7*lr*Th^5*b^4 + 21*lr*Th^4*b^3*psi0 + 21*lr*Th^3*b^2*psi0^2 + 7*lr*Th^2*b*psi0^3))/(42*Th^8);
        dy=-(t^2*(6*lr*Th^6*b^2*psi0 + 3*V*Th^6*b*psi0^2 - 6*V*Th^6*b - 6*lr*Th^5*b*psi0^2 + 12*lr*Th^5*b - 6*lr*Th^4*psi0^3 + 12*lr*Th^4*psi0))/(12*Th^6) + (t^6*(- 3*V*Th^4*b^3 + 6*lr*Th^3*b^3 - 6*V*Th^3*b^2*psi0 + 18*lr*Th^2*b^2*psi0 - 3*V*Th^2*b*psi0^2 + 18*lr*Th*b*psi0^2 + 6*lr*psi0^3))/(36*Th^6) - (t*(V*Th^6*psi0^3 + 3*b*lr*Th^6*psi0^2 - 6*V*Th^6*psi0 - 6*b*lr*Th^6))/(6*Th^6) + (t^7*(V*Th^3*b^3 + 3*V*Th^2*b^2*psi0 + 3*V*Th*b*psi0^2 + V*psi0^3))/(42*Th^6) - (t^4*(V*Th^6*b^3 - 12*lr*Th^5*b^3 - 6*V*Th^5*b^2*psi0 - 6*V*Th^4*b*psi0^2 + 24*lr*Th^3*b*psi0^2 + 12*lr*Th^2*psi0^3))/(24*Th^6) - (t^5*(- 3*V*Th^5*b^3 + 15*lr*Th^4*b^3 + 30*lr*Th^3*b^2*psi0 + 6*V*Th^3*b*psi0^2 + 15*lr*Th^2*b*psi0^2 + 3*V*Th^2*psi0^3))/(30*Th^6) + (t^3*(- 3*lr*Th^6*b^3 - 3*V*Th^6*b^2*psi0 + 18*lr*Th^5*b^2*psi0 + 3*V*Th^5*b*psi0^2 - 6*V*Th^5*b + 18*lr*Th^4*b*psi0^2 + 3*V*Th^4*psi0^3 - 6*V*Th^4*psi0))/(18*Th^6);

        Z_ff(1)=data.initial_pose.position.x+cos(data.road_heading)*dx-sin(data.road_heading)*dy;
        Z_ff(2)= data.initial_pose.position.y+cos(data.road_heading)*dy+sin(data.road_heading)*dx;
end

function angle=minimize_angle(angle)
while angle>=pi
    angle=angle-2*pi;
end
while angle<-pi
    angle=angle+2*pi;
end
end