clear all;
close all;
clc;

%%
% syms a b c;

pg_x = 100;
pg_y = 100;
pg_theta = 0*pi/180;
pg_u_a = 5;
pg_u_b = 2;
pg_u_a_angle = -30*pi/180;
pg_cos = cos(pg_u_a_angle);
pg_sin = sin(pg_u_a_angle);
a = pg_cos^2/(2*pg_u_a^2) + pg_sin^2/(2*pg_u_b^2);
b = pg_cos*pg_sin*(-1/pg_u_a^2 + 1/pg_u_b^2)/2;
c = pg_sin^2/(2*pg_u_a^2) + pg_cos^2/(2*pg_u_b^2);



%%

% sigxsigy_sq = 1/(4*(a*c - b^2));
% sigxsigy = sqrt(sigxsigy_sq);
% 
% K = sigxsigy_sq;
% delta = 4*K^2*(a + c)^2 - 4*K;
% sig_x = sqrt((2*K*(a + c) - sqrt(delta))/2)
% sig_y = sigxsigy/sig_x
% sin_2a = 2*b / ( 1/(2*sig_y^2) - 1/(2*sig_x^2) );
% an = asin(sin_2a)/2;

k = 4*(a*c - b^2);
var_x = (a + c - sqrt((a + c)^2 - k))/k;
% var_y = 1/(a + c - sqrt((a + c)^2 - k))
% var_y = (a + c + sqrt((a + c)^2 - k))/k
var_y = 1/(k*var_x);
% sin_2a = 2*b / ( 1/(2*var_y) - 1/(2*var_x) )
sin_2a = 4*b/(k*(var_x - var_y));
an = asin(sin_2a)/2;
sig_x = sqrt(var_x)
sig_y = sqrt(var_y)

% sig_x = real(sig_x);
% sig_y = real(sig_y);


%%
% pretty(simplify(sig_x))
% pretty(simplify(sig_y))
% pretty(simplify(an))

sig_x
sig_y
an
