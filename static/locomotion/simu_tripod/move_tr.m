clear all;
close all;
clc;

t = 0:.01:1; % (in seconds)

v = [ 0.15 ; 0]; % (in meters/second)

x0 = [0 ; 0.2]; % (in meters)

x = v * t + x0 * ones(size(t)); % (in meters)

xG = [0.3; 0.05]; % (in meters)

a = atan2(xG(2) - x(2, :), xG(1) - x(1, :)); % (in radians)

figure;
plot(x(1,:), x(2, :))
xlabel('(m)'); ylabel('(m)');

figure;
plot(t, a * 180 / pi);
xlabel('(s)'); ylabel('(deg)');
