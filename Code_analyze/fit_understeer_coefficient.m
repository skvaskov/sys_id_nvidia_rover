load('steady_circle.mat')
l=0.3302;
lr=0.12;
m=7.78;
kus=0.00;
cr=10;

g=9.80655;
w_est=delta.*vx./(l+kus*vx.^2/g);
vy_est=w_est.*(lr-m*vx.^2*lr/(cr*l));

R_est=abs(sqrt(vx.^2+vy_est.^2)./w_est);
figure(1)
subplot(2,1,1)
plot(vx,R,'*')
hold on
plot(vx,R_est,'*')
subplot(2,1,2)
plot(delta,R,'*')
hold on
plot(delta,R_est,'*')