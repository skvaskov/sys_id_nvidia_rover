%start with this, you will need to modify what spreadsheets it pulls data
%from, but it basically reads in excel files and saves the data in .mat
%workspace

clc
close all

trialname='test_steer';
full=['~/Documents/MATLAB/Rover_Data/sys_id_jorge/Data/',trialname];




%% mocap
% %load mocap data
range=1:1000;
moname =[full,'_mocap.csv'];
moexcel=readtable(moname);

time_mocap=table2array(moexcel(range,5))+table2array(moexcel(range,6))*10^(-9);

I=table2array(moexcel(range,12));
K=table2array(moexcel(range,11));
J=table2array(moexcel(range,10));

J=smooth(time_mocap,J)/1000;
K=smooth(time_mocap,K)/1000;
I=smooth(time_mocap,I)/1000;

orientW=table2array(moexcel(range,17));
orientJ=table2array(moexcel(range,14));
orientK=table2array(moexcel(range,15));
orientI=table2array(moexcel(range,16));


orientation_mocap=zeros([3,length(time_mocap)]);
Rot_mocap=cell([1,length(time_mocap)]);
for i=1:length(time_mocap)
    [Rot_mocap{i},orientation_mocap(:,i)]=qGetR([orientW(i) orientI(i) orientJ(i) orientK(i)]);
end


CenterPos=[I';J';K'];
CenterPosS=smooth_3(CenterPos,time_mocap);
if length(time_mocap)>3000
v=get_velocity(CenterPosS(:,1:3000),Rot_mocap,time_mocap(1:3000));
else
 v=get_velocity(CenterPosS,Rot_mocap,time_mocap);   
end

mocapstart=get_start(v(1,:),time_mocap,.2,2,60);

clearvars moexcel moname  v orientW orientI orientJ orientK I J K 


%save([trialnames{it},'_workspacerigid.mat'],'-regexp','^(?!(it|trialnames)$).')


   
    

