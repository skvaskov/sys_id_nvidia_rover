function [xc,r] = get_circle(x,y)
%return center and radius of circle x,y are column vectors
 A = [x.^2+y.^2,x,y,ones(size(x))]; % Set up least squares problem
 [~,~,V] = svd(A,0); % Use economy version sing. value decompos.
 a = V(1,4); b = V(2,4); % Choose eigenvector from V
 c = V(3,4); d = V(4,4); % with smallest eigenvalue
 xc = -b/(2*a); yc = -c/(2*a); % Find center and radius of the
 r = sqrt(xc^2+yc^2-d/a); % circle, a*(x^2+y^2)+b*x+c*y+d=0



end

