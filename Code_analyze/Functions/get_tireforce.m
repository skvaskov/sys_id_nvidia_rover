function [Fyf,Fyr,slipf,slipr] = get_tireforce(delta,vx,vy,w,ay,aw,slipmodel,param,scl)
%vx, vy, w, ay, aw all Nx1 column vectors for
%P=[m,Iz,l,lr,af,ar]
l=param(3);
lr=param(4);
lf=l-lr;
Iz=param(2);
m=param(1);

slipf=zeros(size(delta));
slipr=zeros(size(slipf));
Fyf=zeros(size(slipf));
Fyr=zeros(size(slipr));
for i=1:length(slipf)
    if slipmodel=='small'
       slipf(i)=delta(i)-(vy(i)+lf*w(i))/vx(i);
       slipr(i)=(lr*w(i)-vy(i))/vx(i);
    elseif slipmodel=='large'
    slipf(i)=(vx(i)*sin(delta(i))-(vy(i)+lf*w(i))*cos(delta(i)))/(vx(i)*cos(delta(i))+(vy(i)+lf*w(i))*sin(delta(i)));
       slipr(i)=(lr*w(i)-vy(i))/vx(i);
    elseif slipmodel=='fulll'
    af=param(5);
    slipf(i)=(vx(i)*sin(delta(i))-(vy(i)+lf*w(i))*cos(delta(i))-af*w(i))/(vx(i)*cos(delta(i))+(vy(i)+lf*w(i))*sin(delta(i)));
         ar=param(6);
    slipr(i)=((lr-ar)*w(i)-vy(i))/vx(i);
    elseif slipmodel=='custo'
        af=param(5);
         ar=param(6);
       slipf(i)=delta(i)-(vy(i)+lf*w(i)+af*w(i))/vx(i);
       slipr(i)=(lr*w(i)-vy(i)+ar*w(i))/vx(i);
        
    end
    
    ay(i)=0;
    aw(i)=0;
    b=[scl(1)*(m*ay(i)+m*vx(i)*w(i));scl(2)*Iz*aw(i)];
    A=[scl(1)*cos(delta(i)) scl(1)*1;scl(2)*lf*cos(delta(i)) -scl(2)*lr];
    F=A\b;
    Fyf(i)=F(1);
    Fyr(i)=F(2);
end


end
