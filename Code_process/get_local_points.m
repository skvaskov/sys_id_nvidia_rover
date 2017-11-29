clear all
clc
full='~/Documents/MATLAB/Rover_Data/sys_id_nvidia_rover/Raw_Data/mocapcal_mocap.csv';
excel=readtable(full);

id=table2array(excel(:,7));
x=table2array(excel(:,9));
y=table2array(excel(:,10));
z=table2array(excel(:,11));

marker=;

x_mean=mean(x(id==marker))
y_mean=mean(y(id==marker))
z_mean=mean(z(id==marker))