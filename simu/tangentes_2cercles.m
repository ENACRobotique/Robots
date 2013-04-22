%% test sous MATLAB R2012a
clear all;
close all;
clc;

%% ___ donnees ___

% dim = [3 ; 2];
% A = rand(2, 1).*dim;
% B = rand(2, 1).*dim;

A = [0.9 ; 0.6];
B = [2.1 ; 1.4];
LA = 0.2;
LB = 0.5;

%% ___ calculs ___

% repre (A, t, n)
AB = norm(B - A);
t = (B-A)/AB;
n = [-t(2) ; t(1)];

% tangentes internes
M = (B*LA + A*LB)/(LA + LB); % intersection tangentes internes

st = (LA + LB)/AB;
ct = sqrt(1 - st^2);

MA1 = A - LA*(-st*t + ct*n);
MA2 = A - LA*(-st*t - ct*n);
MB1 = B + LB*(-st*t + ct*n);
MB2 = B + LB*(-st*t - ct*n);

% tangentes externes
P = (A*LB - B*LA)/(LB - LA); % intersections tangentes externes

st = (LB - LA)/AB;
ct = sqrt(1 - st^2);

MA3 = A + LA*(-st*t + ct*n);
MA4 = A + LA*(-st*t - ct*n);
MB3 = B + LB*(-st*t + ct*n);
MB4 = B + LB*(-st*t - ct*n);

%% ___ affichage ___

% coefficients directeurs (y - y_A) = m*(x - x_A)
m1 = (MA1(2) - MB1(2))/(MA1(1) - MB1(1));
m2 = (MA2(2) - MB2(2))/(MA2(1) - MB2(1));
m3 = (MA3(2) - MB3(2))/(MA3(1) - MB3(1));
m4 = (MA4(2) - MB4(2))/(MA4(1) - MB4(1));

% affichage
x = 0:0.1:3;
theta = 0:pi/50:2*pi;
figure; hold on; axis('equal', [0 3 0 2]);
% cercle A et ses points
plot(LA*cos(theta)+A(1), LA*sin(theta)+A(2));
plot(A(1), A(2), 'og', MA1(1), MA1(2), 'og', MA2(1), MA2(2), 'og', MA3(1), MA3(2), 'og', MA4(1), MA4(2), 'og');
text(A(1), A(2), ' A');
text(MA1(1), MA1(2), ' MA1');
text(MA2(1), MA2(2), ' MA2');
text(MA3(1), MA3(2), ' MA3');
text(MA4(1), MA4(2), ' MA4');

% cercle B et ses points
plot(LB*cos(theta)+B(1), LB*sin(theta)+B(2));
plot(B(1), B(2), 'or', MB1(1), MB1(2), 'or', MB2(1), MB2(2), 'or', MB3(1), MB3(2), 'or', MB4(1), MB4(2), 'or');
text(B(1), B(2), ' B');
text(MB1(1), MB1(2), ' MB1');
text(MB2(1), MB2(2), ' MB2');
text(MB3(1), MB3(2), ' MB3');
text(MB4(1), MB4(2), ' MB4');

% tangentes internes
plot(x, m1*(x - MA1(1))+MA1(2));
plot(x, m2*(x - MA2(1))+MA2(2));

% intersection tangentes internes
plot(M(1), M(2), 'ok');
text(M(1), M(2), ' M');

% tangentes externes
plot(x, m3*(x - MA3(1))+MA3(2));
plot(x, m4*(x - MA4(1))+MA4(2));

% intersection tangentes externes
plot(P(1), P(2), 'ok');
text(P(1), P(2), ' P');
