dist = 10;
angle = pi/3;

x = 10; % (cm)
y = -10; % (cm)
theta = pi/4; % (rad)

p1_x = x;
p1_y = y;
p2_x = p1_x;
p2_y = p1_y;
c_r = dist/angle
  l_x = c_r*sin(angle)
  l_y = c_r*(cos(angle)-1)
c_x = x + c_r*sin(theta)
c_y = y + -c_r*cos(theta)
end_x = x + cos(theta)*l_x + -sin(theta)*l_y
end_y = y + sin(theta)*l_x + cos(theta)*l_y

figure;
hold on;
plot(p1_x, p1_y, '+g', p1_x+[0:dist/200:dist/5]*cos(theta), p1_y+[0:dist/200:dist/5]*sin(theta), '-b');
plot(c_x, c_y, 'ob', end_x, end_y, '+r');
axis('equal', [x-1.5*dist x+1.5*dist y-1.5*dist y+1.5*dist]);
