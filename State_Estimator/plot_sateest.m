clear all
close all
clc

path='/Users/seanvaskov/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Code_analyze/State_Estimator/Data/';
tname='straight';

figure
excel=readtable([path,tname,'_traj.csv']);
data=table2array(excel);

subplot(3,2,1)
plot(data(:,2),data(:,3))
hold on
xlabel('X')
ylabel('Y')
subplot(3,2,2)
plot(data(:,1),data(:,4))
hold on
xlabel('time (s)')
ylabel('\psi')
subplot(3,2,3)
plot(data(:,1),data(:,5))
hold on
xlabel('time (s)')
ylabel('\omega')
subplot(3,2,4)
plot(data(:,1),data(:,6))
hold on
xlabel('time (s)')
ylabel('v_x')
subplot(3,2,5)
plot(data(:,1),data(:,7))
hold on
xlabel('time (s)')
ylabel('\delta')

excel=readtable([path,tname,'_input.csv']);
data=table2array(excel);
subplot(3,2,6)
plot(data(:,1),data(:,2))
hold on
plot(data(:,1),data(:,3))
legend('steering','throttle')

excel=readtable([path,tname,'_stateest.csv']);
data=table2array(excel);
subplot(3,2,1)
plot(data(:,2),data(:,3))
hold on
xlabel('X')
ylabel('Y')
subplot(3,2,2)
plot(data(:,1),data(:,4))
hold on
xlabel('time (s)')
ylabel('\psi')
subplot(3,2,3)
plot(data(:,1),data(:,5))
hold on
xlabel('time (s)')
ylabel('\omega')
subplot(3,2,4)
plot(data(:,1),data(:,6))
hold on
xlabel('time (s)')
ylabel('v_x')
subplot(3,2,5)
plot(data(:,1),data(:,7))
hold on
xlabel('time (s)')
ylabel('\delta')


for i=1:5
    subplot(3,2,i)
    legend('data','state estimate')
end

figure
subplot(1,2,1)
excel=readtable([path,tname,'_measimu.csv']);
data=table2array(excel);
plot(data(:,1),data(:,2))
hold on
xlabel('time (s)')
ylabel('\omega')
subplot(1,2,2)
excel=readtable([path,tname,'_measimu.csv']);
data=table2array(excel);
plot(data(:,1),data(:,3))
hold on
plot(data(:,1),data(:,4))
xlabel('time (s)')
ylabel('acc')




