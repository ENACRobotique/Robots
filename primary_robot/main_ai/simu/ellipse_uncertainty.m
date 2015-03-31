clear all;
close all;
clc;

%%
pg_x = 100;
pg_y = 100;
pg_theta = 0*pi/180;
pg_u_a = 10/2;
pg_u_b = 2/2;
pg_u_a_angle = pi/6;
%pg_u_theta = 10*pi/180;
pg_c = cos(pg_u_a_angle);
pg_s = -sin(pg_u_a_angle);

mn_x = 95;
mn_y = 105;
mn_theta = 0*pi/180; % n/a
mn_u_a = 3;
mn_u_b = 5;
mn_u_a_angle = 0*pi/180;
%mn_u_theta = 0*pi/180;
mn_c = cos(mn_u_a_angle);
mn_s = -sin(mn_u_a_angle);

%%
pg_x = 100;
pg_y = 100;
pg_theta = 0*pi/180;
pg_u_a = 5/2;
pg_u_b = 15/2;
pg_u_a_angle = 10*pi/180;
%pg_u_theta = 1*pi/180;
pg_c = cos(pg_u_a_angle);
pg_s = -sin(pg_u_a_angle);

mn_x = 110;
mn_y = 110;
mn_theta = 0*pi/180; % n/a
mn_u_a = 10;
mn_u_b = 5;
mn_u_a_angle = 0*pi/180;
%mn_u_theta = 0*pi/180;
mn_c = cos(mn_u_a_angle);
mn_s = -sin(mn_u_a_angle);

%%
[x,y] = meshgrid(0:.5:300, 0:.5:200);

pg_d = 1/(2*pi*pg_u_a*pg_u_b)*exp( -( (x - pg_x).^2*(pg_c^2/pg_u_a^2 + pg_s^2/pg_u_b^2) + (y - pg_y).^2*(pg_c^2/pg_u_b^2 + pg_s^2/pg_u_a^2) - 2*(x - pg_x).*(y - pg_y)*pg_s*pg_c*((pg_u_b^2 - pg_u_a^2)/(pg_u_a^2*pg_u_b^2)) ) );
mn_d = 1/(2*pi*mn_u_a*mn_u_b)*exp( -( (x - mn_x).^2*(mn_c^2/mn_u_a^2 + mn_s^2/mn_u_b^2) + (y - mn_y).^2*(mn_c^2/mn_u_b^2 + mn_s^2/mn_u_a^2) - 2*(x - mn_x).*(y - mn_y)*mn_s*mn_c*((mn_u_b^2 - mn_u_a^2)/(mn_u_a^2*mn_u_b^2)) ) );
% see GaussianStatisticsHandout.pdf

d = pg_d.*mn_d;

figure(1); hold on; axis equal; axis([0 300 0 200]);
% surf(x, y, pg_d);
contour(x, y, pg_d)

% figure(2);
% surf(x, y, mn_d);
contour(x, y, mn_d)

% figure(3);
%surf(x, y, d);
contour(x, y, d)
