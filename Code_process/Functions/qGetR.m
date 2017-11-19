function [R,O] = qGetR( Qrotation )
% qGetR: get a 3x3 rotation matrix
% R = qGetR( Qrotation )
% IN: 
%     Qrotation - quaternion describing rotation
% 
% OUT:
%     R - rotation matrix 
%     
% VERSION: 03.03.2012


w = Qrotation( 1 );
i = Qrotation( 2);
j = Qrotation( 3);
k = Qrotation( 4);

Rxx = 1-2*(j^2+k^2);
Rxy = 2*(i*j - k*w);
Rxz = 2*(i*k + j*w);

Ryx = 2*(i*j + k*w);
Ryy = 1-2*(i^2+k^2);
Ryz = 2*(j*k-w*i);

Rzx = 2*(i*k - j*w);
Rzy = 2*(j*k + i*w );
Rzz = 1-2*(i^2+j^2);

R = [ 
    Rxx,    Rxy,    Rxz;
    Ryx,    Ryy,    Ryz;
    Rzx,    Rzy,    Rzz];

O=[atan2(Rzy,Rzz);...
    -asin(Rzx);...
    atan2(Ryx,Rxx)];
end