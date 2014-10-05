clear all;
close all;
clc;

%%
pg_x = 100;
pg_y = 100;
pg_theta = 0*pi/180;
pg_u_a = 5;
pg_u_b = 2;
pg_u_a_angle = -30*pi/180;
pg_cos = cos(pg_u_a_angle);
pg_sin = sin(pg_u_a_angle);

mn_x = 105;
mn_y = 110;
mn_theta = 0*pi/180; % n/a
mn_u_a = 5;
mn_u_b = 10;
mn_u_a_angle = -10*pi/180;
mn_cos = cos(mn_u_a_angle);
mn_sin = sin(mn_u_a_angle);

%%
pg_a = pg_cos^2/(2*pg_u_a^2) + pg_sin^2/(2*pg_u_b^2);
pg_b = pg_cos*pg_sin*(-1/pg_u_a^2 + 1/pg_u_b^2)/2;
pg_c = pg_sin^2/(2*pg_u_a^2) + pg_cos^2/(2*pg_u_b^2);
pg_k = 1/(2*pi*pg_u_a*pg_u_b);

[sigma_x, sigma_y, a] = abc2sigxya(pg_a, pg_b, pg_c);
disp('pg:');
disp(['  sigma_x =  ',num2str(sigma_x),'cm']);
disp(['  sigma_y =  ',num2str(sigma_y),'cm']);
disp(['  angle   =  ',num2str(a*180/pi),'deg']);
% disp(['  max     =  ',num2str(pg_k)]);

mn_a = mn_cos^2/(2*mn_u_a^2) + mn_sin^2/(2*mn_u_b^2);
mn_b = mn_cos*mn_sin*(-1/mn_u_a^2 + 1/mn_u_b^2)/2;
mn_c = mn_sin^2/(2*mn_u_a^2) + mn_cos^2/(2*mn_u_b^2);
mn_k = 1/(2*pi*mn_u_a*mn_u_b);

[sigma_x, sigma_y, a] = abc2sigxya(mn_a, mn_b, mn_c);
disp('mn:');
disp(['  sigma_x =  ',num2str(sigma_x),'cm']);
disp(['  sigma_y =  ',num2str(sigma_y),'cm']);
disp(['  angle   =  ',num2str(a*180/pi),'deg']);
% disp(['  max     =  ',num2str(mn_k)]);

%%
nw_a = pg_a + mn_a;
nw_b = pg_b + mn_b;
nw_c = pg_c + mn_c;
den = nw_a*nw_c - nw_b^2;
G = -(mn_a*(mn_x - pg_x) + mn_b*(mn_y - pg_y));
H = -(mn_b*(mn_x - pg_x) + mn_c*(mn_y - pg_y));
nw_x = (H*nw_b - G*nw_c) / den + pg_x;
nw_y = (G*nw_b - H*nw_a) / den + pg_y;

% this is the true nw_k after the multiplication of pg and mn
% => its integral is not unitary
nw_k = pg_k*mn_k*exp((nw_a*(nw_x - pg_x)^2 + 2*nw_b*(nw_x - pg_x)*(nw_y - pg_y) + nw_c*(nw_y - pg_y)^2) - (mn_a*(mn_x - pg_x)^2 + 2*mn_b*(mn_x - pg_x)*(mn_y - pg_y) + mn_c*(mn_y - pg_y)^2));
% this is the nw_k with gives an integral of 1
% nw_k = sqrt(nw_a*nw_c - nw_b^2)/pi;

[sigma_x, sigma_y, a] = abc2sigxya(nw_a, nw_b, nw_c);
disp('nw:');
disp(['  sigma_x =  ',num2str(sigma_x),'cm']);
disp(['  sigma_y =  ',num2str(sigma_y),'cm']);
disp(['  angle   =  ',num2str(a*180/pi),'deg']);
% disp(['  max     =  ',num2str(nw_k)]);

[x,y] = meshgrid(80:.5:120, 80:.5:120);
pg_d = pg_k*exp(-(pg_a*(x - pg_x).^2 + 2*pg_b*(x - pg_x).*(y - pg_y) + pg_c*(y - pg_y).^2));
mn_d = mn_k*exp(-(mn_a*(x - mn_x).^2 + 2*mn_b*(x - mn_x).*(y - mn_y) + mn_c*(y - mn_y).^2));
nw_d = nw_k*exp(-(nw_a*(x - nw_x).^2 + 2*nw_b*(x - nw_x).*(y - nw_y) + nw_c*(y - nw_y).^2));

%% 3D
figure(1); hold on;
title('3D');

surf(x, y, pg_d); alpha(0.4)
surf(x, y, mn_d); alpha(0.4)
surf(x, y, nw_d);

%% 2D
figure(2); hold on;
axis equal; axis([0 300 0 200]);
title('2D');

contour(x, y, pg_d)
contour(x, y, mn_d)
% contour(x, y, d)
contour(x, y, nw_d);
plot(nw_x, nw_y, '+r');
