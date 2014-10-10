clear all;
close all;
clc;

A = dlmread('check_arc_log.csv', ';', 1, 0);
% cx;cy;r;dir;sx;sy;ex;ey;wx;wy;nx;ny

A = A(3:end,:);

cx = A(:,1);
cy = A(:,2);
r = A(:,3);
dir = A(:,4);
sx = A(:,5);
sy = A(:,6);
ex = A(:,7);
ey = A(:,8);
wx = A(:,9);
wy = A(:,10);
nx = A(:,11);
ny = A(:,12);

theta = 0:pi/200:2*pi;
figure;axis equal;axis([0 300 0 200]);hold on;
for i = 1:size(A,1)
    plot(cx(i)+r(i)*cos(theta), cy(i)+r(i)*sin(theta));
end
plot(cx,cy,'*')
plot(sx,sy,'*')
plot(ex,ey,'*')
plot(wx,wy,'*')
plot(nx,ny,'*')
