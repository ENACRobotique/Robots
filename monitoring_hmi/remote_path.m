dist = 10;
angle = pi/3;

x = 10; % (cm)
y = -10; % (cm)
theta = pi/4; % (rad)


strt_x = x
strt_y = y
c_r = dist/angle
  l_x = c_r*sin(angle)
  l_y = c_r*(cos(angle)-1)
c_x = x + c_r*sin(theta)
c_y = y + -c_r*cos(theta)
end_x = x + cos(theta)*l_x + -sin(theta)*l_y
end_y = y + sin(theta)*l_x + cos(theta)*l_y

figure;
plot(strt_x, strt_y, '+k', c_x, c_y, '*b', end_x, end_y, '+r');
axis('equal', [x-2*dist x+2*dist y-2*dist y+2*dist]);
