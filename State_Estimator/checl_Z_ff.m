
t=0:0.1:2;
Z_ff=zeros(6,length(t));
data=struct();
data.initial_pose.orientation=pi/2-0.2;
data.initial_pose.position.x=0;
data.initial_pose.position.y=0;
data.b=0;
data.V=2;
data.Th=2;
data.road_heading=pi/2;

for i=1:length(t)
    Z_ff(:,i)=get_Z_ff(t(i),data);
end

figure(1)
subplot(2,2,1)
plot(Z_ff(1,:),Z_ff(2,:))
subplot(2,2,2)
plot(t,Z_ff(3,:))
subplot(2,2,3)
plot(t,Z_ff(4,:))
subplot(2,2,4)
plot(t,Z_ff(6,:))