clear all
close all
clc

covars_imu = zeros(3,3);
name='imu_noise_yr_ax_ay_11_17';   

load('steeringdata.mat')
load('throttledata.mat')
data=[wheelangle_trials,data1_11_17,data2_11_17];
    
    sz=size(data);
    all_imumeas=[];
    all_momeas=[];
    all_err_imu=[];

 l=.3302;
 lr=0.12;
    
    n_imu=0;
    
for j=1:sz(2)    
    
   
    
    % mocap info
    m_time = data(j).no.mocap.time;
    m_psi = data(j).no.mocap.orientation(3,:);
    m_vx = data(j).no.mocap.velocity(1,:);
    m_vy = data(j).no.mocap.velocity(2,:);
    m_yr = data(j).no.mocap.angular_velocity(3,:);
    m_ax = data(j).no.mocap.local_accel_smooth(1,:);
    m_ay = data(j).no.mocap.local_accel_smooth(2,:);
    
    %imuinfo
    i_time=data(j).no.imu.time;
    i_yr=data(j).no.imu.angular_velocity(3,:);
    i_ax=data(j).no.imu.global_accel_smooth(1,:);
    i_ay=data(j).no.imu.global_accel_smooth(2,:);
    i_vx=intmeas(i_ax,i_time);
    i_psi=data(j).no.imu.orientation(3,:);
    
    % inputs
    in_time=data(j).no.input.command.time;
    thro = data(j).no.input.command.throttle;
    ster = data(j).no.input.command.steering;
    in=[ster thro];
    
   
     meas_ax=interp1(m_time',(m_ax-m_vy.*m_yr)',i_time');
     meas_ay=interp1(m_time',(m_ay+m_vx.*m_yr)',i_time');
     meas_yr=interp1(m_time',m_yr',i_time');
     
     
    
    imumeas=[i_yr',i_ax',i_ay'];
    momeas=[meas_yr,meas_ax,meas_ay];
    imumeas(isnan(momeas(:,1)),:)=[];
    momeas(isnan(momeas(:,1)),:)=[];
    err_imu=momeas-imumeas;
    
    
    
    covar_imu =  err_imu.' * err_imu;
    
 
    covar_imu = covar_imu / (length(i_time)-1);
    
    covars_imu = (covars_imu*n_imu + covar_imu*(length(i_time)-1))/(n_imu+(length(i_time)-1));
   
    
    n_imu=n_imu+(length(i_time)-1);
  
    
  
    all_imumeas=[all_imumeas;imumeas];
    all_momeas=[all_momeas;momeas];
    all_err_imu=[all_err_imu;err_imu];
end


   figure
    title('Errors')
    hold on
    plot(all_err_imu(:,1), 'm.')
    plot(all_err_imu(:,2), 'g.')
   plot(all_err_imu(:,3), 'b.')

   legend('\omega','a_x','a_y')
   
    
        
  figure
    subplot(3,1,2)
    plot(all_momeas(:,2))
    hold on
    plot(all_imumeas(:,2))
    legend('act','meas')
    title('Actual vs Measured: a_x')

    subplot(3,1,1)
    plot(all_momeas(:,1))
    hold on
    plot(all_imumeas(:,1))
    legend('act','meas')
    title('Actual vs Measured: \omega')
        subplot(3,1,3)
    plot(all_momeas(:,3))
    hold on
    plot(all_imumeas(:,3))
    legend('act','meas')
    title('Actual vs Measured: a_y')
    

    
disp('imu covariance (\omega a_x a_y): ')
disp(covars_imu)

% clearvars -except name n* covars* data all*
% save([name,'.mat'])
