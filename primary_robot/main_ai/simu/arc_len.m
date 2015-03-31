function [ len ] = arc_len( in, c, r, out )
%ARC_LEN(in, c, r, out) Computes lenght of arc on a circle
%   in:  the input point on the circle
%   c:   the center of the circle
%   r:   the radius of the circle
%   out: the output point on the circle
%   len: length of the arc

v1 = in - c
v3 = out - c
dot = v1(1)*v3(1) + v1(2)*v3(2)
cross = v1(1)*v3(2) - v1(2)*v3(1)

theta = acos(dot/(r*r));

if(r > 0) % CW
    if(cross > 0)
        theta = theta + pi;
    end
else
    if(cross < 0)
        theta = theta + pi;
    end
end

len = abs(theta*r);
end
