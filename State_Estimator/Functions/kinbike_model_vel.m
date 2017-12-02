function state = kinbike_model_vel(state, input, timestep)
    m=7.78;
    Iz=.2120;
    l=.3302;
    lr=.12;
    mo=(m*lr^2+Iz)/l^2;

    u1=input(1);%steering channel
    u0=input(2);%throttle channel
    

   w_prev=state(1);
    vx_prev=state(2);

    delta_prev=state(3);
    
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
    
vxdot=(Frx-mo*tan(delta_prev)/cos(delta_prev)^2*delta_dot*vx_prev)/(m+mo*tan(delta_prev)^2);
    
    
    
    dzdt=[vxdot/l*tan(delta_prev)+vx_prev/l*delta_dot/cos(delta_prev)^2;...
          vxdot;...
          delta_dot];
      

      state(1)=w_prev+timestep*dzdt(1);
      state(2)=vx_prev+timestep*dzdt(2);
      state(3)=delta_prev+timestep*dzdt(3);
      
end