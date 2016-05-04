function [ rot ] = rot( rx, ry, rz )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    Rx = [1 0       0;
          0 cos(rx) sin(rx);
          0 -sin(rx) cos(rx)];
    
    Ry = [cos(ry) 0 -sin(ry);
          0       1  0      ;
          sin(ry) 0 cos(ry)];
  
    Rz = [cos(rz)  sin(rz) 0;
          -sin(rz) cos(rz) 0;
          0         0      1];
      
    rot = inv(Rz*Ry*Rx);
end

