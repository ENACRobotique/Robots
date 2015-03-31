close all;
clear all;

in = [281.86;87.52]%[281.86;87.52];
out = [262.50;111.86]%[238.14;92.5];
r = 22;
% c = [260;90];
% 
% norm(in-c)-abs(r)
% norm(out-c)-abs(r)
% 
% len = arc_len(in, c, r, out)
% abs(2*pi*r)

%r = 22;
alpha = pi/3 %:pi/10:2*pi-pi/10;
c = [260;90];
in = c*ones(size(alpha)) + abs(r)*[cos(alpha);sin(alpha)];
%out = [238;90]*ones(size(alpha));
out = out*ones(size(alpha))

len = arc_len_v(in, c*ones(size(alpha)), r, out)

a = 0:pi/1000:2*pi;
figure;hold on;axis equal;
plot(c(1)+r*cos(a), c(2)+r*sin(a));
plot(out(1,:), out(2,:), 'ok');
plot(in(1,:), in(2,:), '*g');
text(in(1,:), in(2,:), num2str(alpha'*180/pi))

figure;hold on;
plot(alpha*180/pi,len,'*');
plot(alpha*180/pi, 2*pi*r*ones(size(alpha)))
abs(2*pi*r)