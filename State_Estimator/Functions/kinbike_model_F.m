function state = kinbike_model_F(state, input, timestep)
    m=7.78;
    Iz=.2120;
    l=.3302;
    lr=.12;
    mo=(m*lr^2+Iz)/l^2;
    mu=0.4;
    g=9.86055;
    u1=input(1);%steering channel
    u0=input(2);%throttle channel
    
   x_prev=state(1);
    y_prev=state(2);
    psi_prev=state(3);
    vx_prev=state(4);
    vy_prev=state(5);
    w_prev=state(6);
    delta_prev=state(7);

    
    %steering 
    cs1= 0.224314009055080;
    cs2= -0.008867066788855;
    
    delta_des=cs1*u1+cs2;
    
    %driving force input(2)
    if u0<=-0.35
        cm_accel=[-12.5810995587748;-33.0170773577599;4.33920832891501;20.3041178298046 ;0.156420898500981;4.20678380627274;10.2828808092518;-0.610920415224012];
        Frx=[1 u0 vx_prev vx_prev*u0 vx_prev^2 u0^2 vx_prev*u0^2 w_prev^2]*cm_accel;
    elseif u0>0.0
        cm_brake=[ -4.11177295309464;-15.1817204116634; 5.22364002070909];
        Frx=[1 vx_prev vx_prev^2]*cm_brake;
    else
        if vx_prev>0.05
            cm_coast=[-5.55660998280113; -13.8953541919073; -2.47286920126272;0.480990612787014];
             Frx=-[1 u0 vx_prev vx_prev^2]*cm_coast;
        elseif vx_prev<0.05       
             Frx=0.5;
        else 
            Frx=0;
        end
    end
    
    kst= 4.300730919846748;
    delta_dot=kst*(delta_des-delta_prev);
    
     Fry=(m/l*tan(delta_prev)*(1-lr/l)*vx_prev^2-(mo-m*lr/l)/(m+mo*tan(delta_prev)^2)*(Frx*tan(delta_prev)+m*delta_dot*vx_prev/cos(delta_prev)^2));
   Ffy=1/cos(delta_prev)*(m*lr/l^2*vx_prev^2*tan(delta_prev)+mo/(m+mo*tan(delta_prev^2))*(Frx*tan(delta_prev)+m*delta_dot*vx_prev/cos(delta_prev)^2));
   Fzf=lr/l*m*g;
   Fzr=(l-lr)/l*m*g;

   if abs(Ffy)>mu*Fzf
        Ffy=sign(Ffy)*mu*Fzf;
    end
    if sqrt(Fry^2+Frx^2)>mu*Fzr
        if Frx~=0
        theta=atan(Fry/Frx);
        Fry=sin(theta)*mu*Fzr;
        Frx=cos(theta)*mu*Fzr;
        else
            Frx=0;
            Fry=sign(Fry)*mu*Fzr;
        end
    end
    
 dzdt=[(vx_prev)*cos(psi_prev)-vy_prev*sin(psi_prev);...
          (vx_prev)*sin(psi_prev)+vy_prev*cos(psi_prev);...
          w_prev;...
          1/m*(Frx-Ffy*sin(delta_prev)+m*vy_prev*w_prev);...
            1/m*(Fry+Ffy*cos(delta_prev)-m*vx_prev*w_prev);...
              1/Iz*(Ffy*(l-lr)*cos(delta_prev)-Fry*lr);...
                delta_dot];
      
      state(1)=x_prev+timestep*dzdt(1);
      state(2)=y_prev+timestep*dzdt(2);
      state(3)=psi_prev+timestep*dzdt(3);
      state(4)=vx_prev+timestep*dzdt(4);
      state(5)=vy_prev+timestep*dzdt(5);
      state(6)=w_prev+timestep*dzdt(6);
      state(7)=delta_prev+timestep*dzdt(7);
end