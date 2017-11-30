function state = kinbike_model(state, input, timestep)
    m=7.78;
    Iz=.2120;
    l=.3302;
    lr=.12;
    mo=(m*lr^2+Iz)/l^2;

    u1=input(1);%steering channel
    u0=input(2);%throttle channel
    
   x_prev=state(1);
    y_prev=state(2);
    psi_prev=state(3);
   w_prev=state(4);
    vx_prev=state(5);

    delta_prev=state(6);
    
    %steering 
    cs1= 0.224314009055080;
    cs2= -0.008867066788855;
    
    delta_des=cs1*u1+cs2;
    
    %driving force input(2)
    if u0<=-0.35
        cm_accel=[-26.8598346219134;-87.6546485543967;3.14128477268640;35.5572000100370 ; 0.7595495751754 ;-48.4489780846895 ;15.3781821037200;-0.474095491113532];
        Frx=[1 u0 vx_prev vx_prev*u0 vx_prev^2 u0^2 vx_prev*u0^2 w_prev^2]*cm_accel;
    elseif u0>0.0
        cm_brake=[-4.16959507422809;-15.1515686571814;3.564802758464205];
        Frx=[1 vx_prev vx_prev^2]*cm_brake;
    else
        if vx_prev>0.05
            cm_coast=[-5.64256417783774;-13.9194041124684;-2.30808321006301;0.407655664039037];
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
    
    
    
    dzdt=[vx_prev*cos(psi_prev)-lr*w_prev*sin(psi_prev);...
          vx_prev*sin(psi_prev)+lr*w_prev*cos(psi_prev);...
          w_prev;...
          vxdot/l*tan(delta_prev)+vx_prev/l*delta_dot/cos(delta_prev)^2;...
          vxdot;...
          delta_dot];
      
      state(1)=x_prev+timestep*dzdt(1);
      state(2)=y_prev+timestep*dzdt(2);
      state(3)=psi_prev+timestep*dzdt(3);
      state(4)=w_prev+timestep*dzdt(4);
      state(5)=vx_prev+timestep*dzdt(5);
      state(6)=delta_prev+timestep*dzdt(6);
      
end