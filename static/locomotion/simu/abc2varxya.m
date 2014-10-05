function [ var_x, var_y, an ] = abc2varxya( a, b, c )
%ABC2VARXYA Converts quadratic form coefficients to rotated 2D gaussian
%   input:  f(x, y) = ax² + 2bxy + cy²
%   output: f(x, y) = X²/(2*var_x) + Y²/(2*var_y)
%               where: X = cos(an)*x - sin(an)*y
%                      Y = sin(an)*x + cos(an)*y

k = 4*(a*c - b^2); % 1/(var_x*var_y)
var_x = (a + c - sqrt((a + c)^2 - k))/k;
% var_y = 1/(a + c - sqrt((a + c)^2 - k))
% var_y = (a + c + sqrt((a + c)^2 - k))/k
var_y = 1/(k*var_x);
% sin_2a = 2*b / ( 1/(2*var_y) - 1/(2*var_x) )
sin_2a = 4*b/(k*(var_x - var_y));
an = asin(sin_2a)/2;

end
