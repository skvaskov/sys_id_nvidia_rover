function [velS] = smooth_3(vel,time)
%input velocity points 3xn vector with coressponding time points 1xn
%output smooth 3xxn velocity vector smooth 

velS=[smooth(time,vel(1,:))';...
        smooth(time,vel(2,:))';...
        smooth(time,vel(3,:))'];


end

