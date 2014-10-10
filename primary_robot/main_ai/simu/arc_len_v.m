function [ len ] = arc_len_v( in, c, r, out )
%ARC_LEN_V(in, c, r, out) Computes lenght of arc on a circle
%   in:  the input point on the circle
%   c:   the center of the circle
%   r:   the radius of the circle
%   out: the output point on the circle
%   len: length of the arc

v1 = in - c;
v3 = out - c;
dot = v1(1,:).*v3(1,:) + v1(2,:).*v3(2,:);
cross = v1(1,:).*v3(2,:) - v1(2,:).*v3(1,:);

theta = acos(dot/(r*r));

if(r > 0) % CW
    theta(cross > 0) = theta(cross > 0) + pi;
    if(cross>0)
        disp('cross>0')
    end
%     if(cross > 0)
%         theta = theta + pi;
%     end
else % CCW
    theta(cross < 0) = theta(cross < 0) + pi;
    if(cross<0)
        disp('cross<0')
    end
%     if(cross < 0)
%         theta = theta + pi;
%     end
end

len = abs(theta*r);
end
