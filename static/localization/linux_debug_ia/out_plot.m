clear all;
close all;
clc;

A = dlmread('Debug/out.csv', ';');

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
PercConv = reshape(A(:, 7), sr, sc);

figure; surf(X, Y, NbCri); title('Number of criteria evaluation');
figure; surf(X, Y, min(MeanU, .05)); title('Mean uncertainty saturated to .05 (m)');
figure; surf(X, Y, min(MaxU, .05)); title('Max uncertainty saturated to .05 (m)');
figure; surf(X, Y, StdDev); title('Standard Deviation (m)');
figure; surf(X, Y, min(StdDev./MeanU*100, 100)); title('Standard Deviation saturated to 100 (%)');
figure; surf(X, Y, min(PercConv*100, 100)); title('Percentage of convergence saturated to 100 (%)');

figure;
[NN,XX] = hist(MeanU(:), 50);
subplot(2,1,1); bar(XX, NN); title('histogram of mean uncertainty (m)');
for i=2:size(NN,2); NN(i)=NN(i)+NN(i-1); end; NN=NN/NN(end);
subplot(2,1,2); bar(XX,NN); title('cumulative');

%NbCri = conv2(NbCri, ones(4,4)/16, 'same');
%figure; surf(X, Y, NbCri); title('Number of criteria evaluation (filtered)');
