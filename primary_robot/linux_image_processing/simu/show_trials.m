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

ct = cos(theta);
st = sin(theta);

%%
figure;
hold on;
plot(x, y, 'r-*');
l = 1;
% plot([x x+l*ct(:)]', [y y+l*st(:)]')
for i = iter
    text(v(i,1), v(i,2)+l/3, ['v', num2str(i)])
%     text(v(i,1), v(i,2)-l/3, [num2str(v(i,3)*180/pi)])
end

%%
figure;
subplot(4, 1, 1); plot(iter, x);
subplot(4, 1, 2); plot(iter, y);
subplot(4, 1, 3); plot(iter, theta*180/pi);
subplot(4, 1, 4); plot(iter, e);
