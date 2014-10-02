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
pg_cos = cos(pg_u_a_angle);
pg_sin = sin(pg_u_a_angle);

mn_x = 95;
mn_y = 105;
mn_theta = 0*pi/180; % n/a
mn_u_a = 3;
mn_u_b = 5;
mn_u_a_angle = 0*pi/180;
%mn_u_theta = 0*pi/180;
mn_cos = cos(mn_u_a_angle);
mn_sin = sin(mn_u_a_angle);

%%
pg_x = 100;
pg_y = 100;
pg_theta = 0*pi/180;
pg_u_a = 5/2;
pg_u_b = 15/2;
pg_u_a_angle = 30*pi/180;
%pg_u_theta = 1*pi/180;
pg_cos = cos(pg_u_a_angle);
pg_sin = sin(pg_u_a_angle);

mn_x = 110;
mn_y = 110;
mn_theta = 0*pi/180; % n/a
mn_u_a = 10;
mn_u_b = 5;
mn_u_a_angle = 0*pi/180;
%mn_u_theta = 0*pi/180;
mn_cos = cos(mn_u_a_angle);
mn_sin = sin(mn_u_a_angle);

%%

pg_a = pg_cos^2/(2*pg_u_a^2) + pg_sin^2/(2*pg_u_b^2);
pg_b = pg_cos*pg_sin*(-1/pg_u_a^2 + 1/pg_u_b^2)/2;
pg_c = pg_sin^2/(2*pg_u_a^2) + pg_cos^2/(2*pg_u_b^2);
pg_k = 1/(2*pi*pg_u_a*pg_u_b);

mn_a = mn_cos^2/(2*mn_u_a^2) + mn_sin^2/(2*mn_u_b^2);
mn_b = mn_cos*mn_sin*(-1/mn_u_a^2 + 1/mn_u_b^2)/2;
mn_c = mn_sin^2/(2*mn_u_a^2) + mn_cos^2/(2*mn_u_b^2);
mn_k = 1/(2*pi*mn_u_a*mn_u_b);
dx = mn_x - pg_x;
dy = mn_y - pg_y;

G = -(mn_a*dx + mn_b*dy);
H = -(mn_b*dx + mn_c*dy);

nw_a = pg_a + mn_a;
nw_b = pg_b + mn_b;
nw_c = pg_c + mn_c;
den = nw_a*nw_c - nw_b^2;
nw_x = (H*nw_b - G*nw_c) / den + pg_x;
nw_y = (G*nw_b - H*nw_a) / den + pg_y;
nw_k = pg_k*mn_k*exp((nw_a*(nw_x - pg_x)^2 + 2*nw_b*(nw_x - pg_x)*(nw_y - pg_y) + nw_c*(nw_y - pg_y)^2) - (mn_a*(mn_x - pg_x)^2 + 2*mn_b*(mn_x - pg_x)*(mn_y - pg_y) + mn_c*(mn_y - pg_y)^2));

[x,y] = meshgrid(0:.3:300, 0:.3:200);
pg_d = pg_k*exp(-(pg_a*(x - pg_x).^2 + 2*pg_b*(x - pg_x).*(y - pg_y) + pg_c*(y - pg_y).^2));
mn_d = mn_k*exp(-(mn_a*(x - mn_x).^2 + 2*mn_b*(x - mn_x).*(y - mn_y) + mn_c*(y - mn_y).^2));
nw_d = nw_k*exp(-(nw_a*(x - nw_x).^2 + 2*nw_b*(x - nw_x).*(y - nw_y) + nw_c*(y - nw_y).^2));

d = pg_d.*mn_d;

%%
figure(1); hold on; axis equal; axis([0 300 0 200]);
% surf(x, y, pg_d);
contour(x, y, pg_d)

% figure(2);
% surf(x, y, mn_d);
contour(x, y, mn_d)

% figure(3);
% surf(x, y, d);
contour(x, y, d)
contour(x, y, nw_d)

plot(nw_x, nw_y, 'ok');

%%
[x,y] = meshgrid(0:.5:300, 0:.5:200);

pg_d = 1/(2*pi*pg_u_a*pg_u_b)*exp( -( (x - pg_x).^2*(pg_cos^2/pg_u_a^2 + pg_sin^2/pg_u_b^2) + (y - pg_y).^2*(pg_cos^2/pg_u_b^2 + pg_sin^2/pg_u_a^2) - 2*(x - pg_x).*(y - pg_y)*pg_sin*pg_cos*(1/pg_u_a^2 - 1/pg_u_b^2) )/2 );
mn_d = 1/(2*pi*mn_u_a*mn_u_b)*exp( -( (x - mn_x).^2*(mn_cos^2/mn_u_a^2 + mn_sin^2/mn_u_b^2) + (y - mn_y).^2*(mn_cos^2/mn_u_b^2 + mn_sin^2/mn_u_a^2) - 2*(x - mn_x).*(y - mn_y)*mn_sin*mn_cos*(1/mn_u_a^2 - 1/mn_u_b^2) )/2 );
% see GaussianStatisticsHandout.pdf

d = pg_d.*mn_d;

figure(2); hold on; axis equal; axis([0 300 0 200]);
% surf(x, y, pg_d);
contour(x, y, pg_d)

% figure(2);
% surf(x, y, mn_d);
contour(x, y, mn_d)

% figure(3);
% surf(x, y, d);
contour(x, y, d)
contour(x, y, nw_d)

plot(nw_x, nw_y, 'ok');
