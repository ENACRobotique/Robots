clear all;
close all;
clc;

A = dlmread('out.csv', ';');

sr = A(1,2);
sc = A(1,1);

A(2,:) = [];
A(1,:) = [];

X = reshape(A(:, 1), sr, sc);
Y = reshape(A(:, 2), sr, sc);
NbCri = reshape(A(:, 3), sr, sc);
MeanU = reshape(A(:, 4), sr, sc);
MaxU = reshape(A(:, 5), sr, sc);
StdDev = reshape(A(:, 6), sr, sc);

figure; surf(X, Y, NbCri); title('Number of criteria evaluation');
figure; surf(X, Y, min(MeanU, .05)); title('Mean uncertainty saturated to .05');
figure; surf(X, Y, min(MaxU, .05)); title('Max uncertainty saturated to .05');
figure; surf(X, Y, StdDev./MeanU); title('Standard Deviation (%)');

NbCri = conv2(NbCri, ones(4,4)/100, 'same');

figure; surf(X, Y, NbCri); title('Number of criteria evaluation (filtered)');

