clear all;
close all;
clc;

%% constants describing the position/orientation of the two 2D gaussians
pg_x = 100;
pg_y = 100;
pg_u_a = 5;
pg_u_b = 2;
pg_u_a_angle = [-180:180]*pi/180;

pg_u_a = ones(size(pg_u_a_angle))*pg_u_a;
pg_u_b = ones(size(pg_u_a_angle))*pg_u_b;

%%
[pg_a, pg_b, pg_c] = varxya2abc(pg_u_a.^2, pg_u_b.^2, cos(pg_u_a_angle), sin(pg_u_a_angle))
pg_k = 1./(2*pi*pg_u_a.*pg_u_b);

figure;
hold on;
title('a, b, c')
plot(pg_u_a_angle*180/pi, pg_a, 'r');
plot(pg_u_a_angle*180/pi, pg_b, 'g');
plot(pg_u_a_angle*180/pi, pg_c, 'b');
legend(['a';'b';'c']);