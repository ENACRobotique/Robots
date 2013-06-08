clear all;
close all;
clc;

syms B1x B1y;
syms B2x B2y;
syms B3x B3y;
B1 = [B1x ; B1y];
B2 = [B2x ; B2y];
B3 = [B3x ; B3y];

syms D12 D23 D31;

syms x y;

% distances
syms D1 D2 D3;
syms s_D1 s_D2 s_D3;
d1 = sqrt((x - B1(1)).^2 + (y - B1(2)).^2);
d2 = sqrt((x - B2(1)).^2 + (y - B2(2)).^2);
d3 = sqrt((x - B3(1)).^2 + (y - B3(2)).^2);

% angles
syms A12 A23 A31;
syms s_A12 s_A23 s_A31;
a12 = acos((d1^2 + d2^2 - D12^2)/(2*d1*d2));
a23 = acos((d2^2 + d3^2 - D23^2)/(2*d2*d3));
a31 = acos((d3^2 + d1^2 - D31^2)/(2*d3*d1));

z = (d1 - D1)^2/s_D1^2 + (d2 - D2)^2/s_D2^2 + (d3 - D3)^2/s_D3^2 + (a12 - A12)^2/s_A12^2 + (a23 - A23)^2/s_A23^2 + (a31 - A31)^2/s_A31^2;
% z = 1 - exp(-z);

disp('z_x=');
z_x = diff(z, x);
pretty(z_x)

disp('z_y=');
z_y = diff(z, y);
pretty(z_y)

% matrice jacobienne
% J = -2*(z + 1)*[ (d1 - D1)*(x - B1(1))/(d1 * s_D1^2) + (d2 - D2)*(x - B2(1))/(d2 * s_D2^2) + (d3 - D3)*(x - B3(1))/(d3 * s_D3^2), (d1 - D1)*(y - B1(2))/(d1 * s_D1^2) + (d2 - D2)*(y - B2(2))/(d2 * s_D2^2) + (d3 - D3)*(y - B3(2))/(d3 * s_D3^2) ];
