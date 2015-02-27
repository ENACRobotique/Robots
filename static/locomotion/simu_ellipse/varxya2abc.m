function [ a, b, c ] = varxya2abc( var_x, var_y, ca, sa )
%VARXYA2ABC Converts rotated 2D gaussian to quadratic form coefficients
%   input:  f(x, y) = X²/(2*var_x) + Y²/(2*var_y)
%               where: X = cos(an)*x - sin(an)*y
%                      Y = sin(an)*x + cos(an)*y
%               where: cos(an) = ca
%                      sin(an) = sa
%   output: f(x, y) = ax² + 2bxy + cy²

a = ca.^2./(2*var_x) + sa.^2./(2*var_y);
b = ca.*sa.*(-1./var_x + 1./var_y)/2;
c = sa.^2./(2*var_x) + ca.^2./(2*var_y);

end

