function [A] = e2a(rx,ry,rz,deg)
    if nargin == 3
        deg = false;
    end
    if deg
        rx = deg2rad(rx);
        ry = deg2rad(ry);
        rz = deg2rad(rz);
    end

    Rz = [cos(rz)   sin(rz)  0;
          -sin(rz)   cos(rz)  0;
             0         0     1];

    Ry = [ cos(ry) 0  -sin(ry);
              0    1     0;
           sin(ry) 0  cos(ry)];
      
    Rx = [1      0       0;
          0  cos(rx) sin(rx);
          0  -sin(rx)  cos(rx)];

    A = Rx*Ry*Rz;
end