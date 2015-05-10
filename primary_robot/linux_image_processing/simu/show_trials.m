clc;
clear all;
close all;

%%
v = dlmread('../out_trials.csv', ',', 1, 0);

iter = 1:size(v,1);

x = v(:,1);
y = v(:,2);
theta = v(:,3);
e = v(:,4);

[Y, I] = min(e);
best_I = I
best_X = v(I,1)
best_Y = v(I,2)
best_Theta = v(I,3)*180/pi
best_E = v(I,4)

ct = cos(theta);
st = sin(theta);

%%
figure;
hold on;
plot3(x, y, theta, 'r-*');
for i = iter
    text(v(i,1), v(i,2), v(i,3), ['v', num2str(i)])
end

%%
figure;
subplot(4, 1, 1); plot(iter, x);
subplot(4, 1, 2); plot(iter, y);
subplot(4, 1, 3); plot(iter, theta*180/pi);
subplot(4, 1, 4); plot(iter, e);
