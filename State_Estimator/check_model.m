clear all
close all
clc
load('steeringdata.mat')
load('throttledata.mat')
load('joystickdata.mat')
covars = zeros(3,3);

 name='kinematic_covariance_12_1';   
 
data=[circles_11_29,turns_11_29,data2_11_17,data1_11_28,decel_11_28];
%data=[data_11_17];
    sz=size(data);
    all_est=[];
    all_state=[];
    all_err=[];
    all_in=[];
    n=0;
    
for j=1:sz(2)
      
    si= find(data(j).interp.mocap.velocity(1,:)>0.5,1);
    ei= find(data(j).interp.mocap.velocity(1,:)>0.5,1,'last');
    
    if ei>si
    % mocap info
    time = data(j).interp.time(si:ei);
    r_x = data(j).interp.mocap.pos(1, si:ei);
    r_y = data(j).interp.mocap.pos( 2,si:ei);
    r_h = data(j).interp.mocap.orientation(3,si:ei);
    r_vx = data(j).interp.mocap.velocity(1,si:ei);
    r_yr = data(j).interp.mocap.angular_velocity(3,si:ei);
    r_ax = data(j).interp.mocap.local_accel_smooth(1,si:ei);
    
    % inputs
    thro = data(j).interp.input.command.throttle(si:ei);
    ster = data(j).interp.input.command.steering(si:ei);
    in=[ster' thro'];
    
    %estimate wheel angle
    r_delta=atan(0.3302*r_yr./r_vx);
    
    r_state=[ r_yr' r_vx' r_delta'];
    
    
    covar = zeros(3, 3);
    e_state=zeros(ei-si,3);
    
    for i=1:ei-si
        timestep = time(i+1) - time(i);
        e_state(i,:) = kinbike_model_vel(r_state(i,:), in(i,:), timestep);
        err = e_state(i,:) - r_state(i+1,:);
        covar = covar + err.' * err / timestep;
    end
    if isempty(find(isnan(covar),1))
    covar = covar / (ei - si);
    % disp(1000 * covar(1:4,1:4))
    %disp(covar)
    covars = (covars*n + covar*(ei-si))/(n+(ei-si));
    n=n+(ei-si);
    
    e_state=[r_state(1,:);e_state];
    error=e_state-r_state;
    all_state=[all_state;r_state];
    all_est=[all_est;e_state];
    all_err=[all_err;error];
    all_in=[all_in;in];
    end
    end
end

figure
    title(' Accel, Yaw Rate, & Inputs')
    hold on
    yyaxis left
    plot(all_state(:,1) , 'r')
    plot(all_state(:,2), 'b')
    plot(all_state(:,3), 'k')
    yyaxis right
    plot(all_in(:,2), 'g')
    plot(all_in(:,1), 'm')
      legend('\omega','v_x','delta','throttle','steering')
   figure
    title('Errors')
    hold on
    plot(all_err(:,1), 'bx')
    plot(all_err(:,2), 'kx')
    legend('\omega','v_x')
        
  figure
    subplot(2,1,1)
    plot(all_state(:,1))
    hold on
    plot(all_est(:,1))
    legend('act','est')
    title('Actual vs Estimated: \omega')
    subplot(2,1,2)
    plot(all_state(:,2))
    hold on
    plot(all_est(:,2))
    legend('act','est')
    title('Actual vs Estimated: v_x')
    
disp(covars)
clearvars -except name n covars data all*
save([name,'.mat'])
