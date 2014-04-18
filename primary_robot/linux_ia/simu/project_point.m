%by Seb

clear all;
close all;
clc;

%---données---
xp=-16;
yp=-45;
rc=10;
xc=5;
yc=0;
xf=0.771145;
yf=-9.061831;


theta = 0:0.01:2*pi;
figure;
hold on;

xcercle=rc*cos(theta)+xc;
ycercle=rc*sin(theta)+yc;

plot(xcercle, ycercle)
plot(xp, yp,'+')
plot(xf, yf,'+')
