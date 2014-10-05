function [ sig_x, sig_y, an ] = abc2sigxya( a, b, c )
%ABC2SIGXYA Summary of this function goes here
%   Detailed explanation goes here

sigxsigy_sq = 1/(4*(a*c - b^2));
sigxsigy = sqrt(sigxsigy_sq);

K = sigxsigy_sq;
delta = 4*K^2*(a + c)^2 - 4*K;
sig_x = sqrt((2*K*(a + c) - sqrt(delta))/2);
sig_y = sigxsigy/sig_x;
sin_2a = 2*b / ( 1/(2*sig_y^2) - 1/(2*sig_x^2) );
an = asin(sin_2a)/2;

sig_x = real(sig_x);
sig_y = real(sig_y);

end

