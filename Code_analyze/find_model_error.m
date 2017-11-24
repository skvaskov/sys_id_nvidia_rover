clear all
close all


load('steeringdata.mat')
data=[sturns];
load('throttledata.mat')
%data=[];

l=0.3302;
error=[]; %error speed yaw rate
sz=size(data);
for num=1:sz(2)
    
times=0:0.25:data(num).interp.time(end);
figure
plot(data(num).interp.mocap.pos(1,:),data(num).interp.mocap.pos(2,:))
hold on
for it=times
    idxs=find(data(num).interp.time>=it,1);
    idxe=find(data(num).interp.time<=(it+0.5),1,'last');
    ster_input=@(t)zoh(data(num).no.input.command.time',data(num).no.input.command.steering,t);
    thro_input=@(t)zoh(data(num).no.input.command.time',data(num).no.input.command.throttle,t);
    if data(num).interp.mocap.velocity(1,idxs)>=0.5
        tspan=[data(num).interp.time(idxs), data(num).interp.time(idxe)]
        y0=[data(num).interp.mocap.pos(1,idxs), data(num).interp.mocap.pos(2,idxs),...
            data(num).interp.mocap.orientation(3,idxs), data(num).interp.mocap.velocity(1,idxs),...
            atan(l*data(num).interp.mocap.angular_velocity(3,idxs)/data(num).interp.mocap.velocity(1,idxs))];
        [t,y]=ode45(@(t,y)kinbike_model_ode(t,y,ster_input,thro_input),tspan,y0);
        plot(y(:,1),y(:,2),'r')
        error=[error;sqrt((y(end,1)-data(num).interp.mocap.pos(1,idxe))^2+(y(end,2)-data(num).interp.mocap.pos(2,idxe))^2)];
    end
end
end