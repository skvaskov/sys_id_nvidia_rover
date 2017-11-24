function dzdt = kinbike_model_ode(t,z, ster_input,thro_input)
    m=7.78;
    Iz=.2120;
    l=.3302;
    lr=.12;
    mo=(m*lr^2+Iz)/l^2;

    u1=ster_input(t);%steering channel
    u0=thro_input(t);%throttle channel
    
   x_prev=z(1);
    y_prev=z(2);
    psi_prev=z(3);
    vx_prev=z(4);

    delta_prev=z(5);
    
    %steering 
    cs1= 0.224314009055080;
    cs2= -0.008867066788855;
    
    delta_des=cs1*u1+cs2;
    
    %driving force input(2)
    if u0<=-0.35
        cm_accel=[-32.3444674451266;-107.4936376966535;5.4559263010323;35.5572000100370;0.7595495751754;-48.4489780846895;21.5881438769874];
        Frx=[1 u0 vx_prev vx_prev*u0 vx_prev^2 u0^2 vx_prev*u0^2]*cm_accel;
    elseif u0>=1.0
        cm_brake=[-5.345435991947356;-11.906846985500659;3.564802758464205];
        Frx=[1 vx_prev vx_prev^2]*cm_brake;
    else
        if vx_prev>0
            cm_coast=[-5.767811170782461;0.378942271548532;-0.346210024796956];
             Frx=[1 vx_prev vx_prev^2]*cm_coast;
        elseif vx_prev<0
            cm_coast=[-5.767811170782461;0.378942271548532;-0.346210024796956];
             Frx=-[1 vx_prev vx_prev^2]*cm_coast;
        else 
            Frx=0;
        end
    end
    
    kst= 4.300730919846748;
    delta_dot=kst*(delta_des-delta_prev);
    
vxdot=(Frx-mo*tan(delta_prev)/cos(delta_prev)^2*delta_dot*vx_prev)/(m+mo*tan(delta_prev)^2);
    
    
    
    dzdt=[vx_prev*cos(psi_prev)-lr*vxdot/l*tan(delta_prev)*sin(psi_prev);...
          vx_prev*sin(psi_prev)+lr*vxdot/l*tan(delta_prev)*cos(psi_prev);...
          vx_prev/l*tan(delta_prev);...
          vxdot;...
          delta_dot];
      
      
      
end