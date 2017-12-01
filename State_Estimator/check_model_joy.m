clear all
close all
clc
load('steeringdata.mat')
load('throttledata.mat')
load('joystickdata.mat')
covars = zeros(6,6);

 name='kinematic_covariance_11_30';   
 
%data=[circles_11_29,turns_11_29,data2_11_17,data1_11_28,decel_11_28];
data=[data_11_17];
    sz=size(data);
    all_est=[];
    all_state=[];
    all_err=[];
    all_in=[];
    n=0;
    
for j=1:sz(2)
      
    idxes= find(data(j).interp.mocap.velocity(1,:)>0.2);
  
    l=length(idxes);
    
    % mocap info
    time = data(j).interp.time;
    r_x = data(j).interp.mocap.pos(1, :);
    r_y = data(j).interp.mocap.pos( 2,:);
    r_h = data(j).interp.mocap.orientation(3,:);
    r_vx = data(j).interp.mocap.velocity(1,:);
    r_yr = data(j).interp.mocap.angular_velocity(3,:);
    r_ax = data(j).interp.mocap.local_accel_smooth(1,:);
    
    % inputs
    thro = data(j).interp.input.command.throttle;
    ster = data(j).interp.input.command.steering;
    in=[ster' thro'];
    
    %estimate wheel angle
    r_delta=atan(0.3302*r_yr./r_vx);
    
    r_state=[r_x' r_y' r_h' r_yr' r_vx' r_delta'];
    
    
    covar = zeros(6, 6);
    e_state=zeros(l,6);
    c=1;
    for i=idxes
        timestep = time(i+1) - time(i);
        e_state(c,:) = kinbike_model(r_state(i,:), in(i,:), timestep);
        err = e_state(c,:) - r_state(i+1,:);
        covar = covar + err.' * err / timestep;
        c=c+1;
    end
    if isempty(find(isnan(covar),1))
    covar = covar / (l);
    % disp(1000 * covar(1:4,1:4))
    %disp(covar)
    covars = (covars*n + covar*(l))/(n+(l));
    n=n+(l);
    

    error=e_state-r_state(idxes,:);
    all_state=[all_state;r_state];
    all_est=[all_est;e_state];
    all_err=[all_err;error];
    all_in=[all_in;in];
    end

end

figure
    title(' Accel, Yaw Rate, & Inputs')
    hold on
    yyaxis left
    plot(all_state(:,4) , 'r')
    plot(all_state(:,5), 'b')
    plot(all_state(:,6), 'k')
    yyaxis right
    plot(all_in(:,2), 'g')
    plot(all_in(:,1), 'm')
      legend('\omega','v_x','delta','throttle','steering')
   figure
    title('Errors')
    hold on
    plot(all_err(:,4), 'bx')
    plot(all_err(:,5), 'kx')
    legend('\omega','v_x')
        
  figure
    subplot(2,1,1)
    plot(all_state(:,4))
    hold on
    plot(all_est(:,4))
    legend('act','est')
    title('Actual vs Estimated: \omega')
    subplot(2,1,2)
    plot(all_state(:,5))
    hold on
    plot(all_est(:,5))
    legend('act','est')
    title('Actual vs Estimated: v_x')
    
disp(covars)
clearvars -except name n covars data all*
save([name,'.mat'])
