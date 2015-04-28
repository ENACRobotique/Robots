clear all;
close all;
clc; % clear console

%% load source image
I = imread('src_edges.png');
%figure;imshow(I);title('table 2015');

factor = 4; % (px/cm)

II = double(rgb2gray(I))/255;
%figure;imshow(II);title('table 2015 GRAY');

%% detect edges
J = double(edge(II, 'canny'));
%J = imfilter(II, fspecial('laplacian', 0));

%% create map of distances

if ~exist('dist.mat', 'file')
    E = J;
    D = E;
    SE = [0 1 0;1 1 1;0 1 0]%strel('diamond', 1);
    while min(min(E)) == 0
        E = imdilate(E, SE);

        D = D + E;
    end
    D = (max(max(D)) - D)./factor; % (cm)

    d = D(300:400, 400:500); % extract a star
    figure;
    subplot(1, 2, 1);surf(d);
    subplot(1, 2, 2);imagesc(d);axis equal;

    % save dist image
    save('dist', 'D');

    % pt = D(215, 147:180);
    % figure;plot(pt);
else
    load('dist.mat')
end

DD = D/max(max(D));
imwrite(DD, 'dist.png', 'bitdepth', 16);

%% get propability from distance

low_dens = 1/15; % (/cm²)
low_thr = 15; % (cm)

high_dens = 1.2/1; % (/cm²)
high_thr = 0;

P = high_dens + (high_dens - low_dens) .* (D - high_thr) ./ (high_thr - low_thr);

P(D >= low_thr) = low_dens;
P(D <= high_thr) = 0;

P = min(P./(factor^2), 1);

figure;imshow(P)

%p = P(300:400, 400:500); % extract a star
%figure;
%subplot(1, 2, 1);surf(p);
%subplot(1, 2, 2);imagesc(p);axis equal;

%pt = P(215, :);
%figure;plot(pt);

imwrite(P, 'prob.png', 'bitdepth', 16);

%% generate test points

Q = false(size(P));
Q(P > rand(size(P))) = true;

sum(sum(Q))

%figure;imshow(Q);

imwrite(Q, 'tp_nb.png');

%% create color ones

i = imread('src_colors.png');
i = double(i)/255;

% j = rgb2gray(i);
% b = j < 0.1;
% ir = i(:, :, 1); ir(b) = 1; i(:, :, 1) = ir;
% ig = i(:, :, 2); ig(b) = 1; i(:, :, 2) = ig;
% ib = i(:, :, 3); ib(b) = 1; i(:, :, 3) = ib;

a = zeros(size(i));
a(:,:,1) = Q;
a(:,:,2) = Q;
a(:,:,3) = Q;

A = i.*a;

% b = rgb2gray(A) < 0.1;
% ir = A(:, :, 1); ir(b) = 1; A(:, :, 1) = ir;
% ig = A(:, :, 2); ig(b) = 1; A(:, :, 2) = ig;
% ib = A(:, :, 3); ib(b) = 1; A(:, :, 3) = ib;

%figure;imshow(A);

imwrite(A, 'tp_col.png');

%% mask-out the result

k = rgb2gray(imread('src_mask.png'));
k = k < 128;

c = zeros(size(k));
c(:,:,1) = k;
c(:,:,2) = k;
c(:,:,3) = k;

B = A.*c;

figure;imshow(B);

imwrite(B, 'tp.png');

%% write testpoints to file

HSV = rgb2hsv(i);
H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);

[row, col] = find(Q);
h = H(Q);
s = S(Q);
v = V(Q);
dist = D(Q);
dens = P(Q) .* factor^2;

M = [(col - 30 + 1)./factor, (size(i,1) - row + 1  - 30 + 1)./factor, h, s, v, dist, dens];

dlmwrite('testpoints.csv', M);

%%
% %j = J(100:200, 100:200);
% %figure;surf(j);
% 
% sigma = 10
% %h = fspecial('disk', 20);
% h = fspecial('gaussian', 2*3*sigma, sigma);
% 
% K = imfilter(J, h);
% 
% 
% k = K(100:200, 100:200);
% figure;
% subplot(1, 2, 1);surf(k);
% subplot(1, 2, 2);imagesc(k);axis equal;
% pt = K(215, 147:180);
% figure;plot(pt);
