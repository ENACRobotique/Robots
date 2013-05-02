clear all;
close all;
clc;

A = dlmread('out.csv', ';');

X = reshape(A(:, 1), 39, 59);
Y = reshape(A(:, 2), 39, 59);
Z = reshape(A(:, 3), 39, 59);
I = reshape(A(:, 4), 39, 59);

figure; surf(X, Y, Z);
figure; surf(X, Y, min(I, .05));

Z = conv2(Z, ones(4,4)/100, 'same');
I = conv2(I, ones(4,4)/100, 'same');

figure; surf(X, Y, Z); title('filtered');
figure; surf(X, Y, I); title('filtered');
