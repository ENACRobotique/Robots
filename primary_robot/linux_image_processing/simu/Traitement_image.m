clear all;
close all;
clc; %clear console

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Constantes %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Angle (°) de rotation du repère cam
        alpha_x = 225*pi/180; %radian
        alpha_y = 0;
        alpha_z = 0; 
    % Coordonnées origine cam dans repère robot
        Ocx_R = 0;
        Ocy_R = 107;
        Ocz_R = 267;
    % distance focale (ps3 eye [2.5; 2.8 mm]
        f = 521.3; %en pixels 
    % altitude plan de surface des feux dans le repère robot
        zM = 200; 
    % Centre de l'image en pixel
        u0 = (0+639)/2; 
        v0 = (0+479)/2; 
    % Coordonnées du pixel dans l'image
        u = u0;
        v = v0;
    % distance focale (ps3 eye [2.5; 2.8 mm]
        f2 = 5; %en pixels 
    % Centre de l'image en pixel
        u20 = 538; 
        v20 = 570; 

        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Marices de rotations
Rx_R_C = [1 0 0; 0 cos(alpha_x) sin(alpha_x); 0 -sin(alpha_x) cos(alpha_x)];
Ry_R_C = [cos(alpha_y) 0 -sin(alpha_y); 0 1 0; sin(alpha_y) 0 cos(alpha_y)];
Rz_R_C = [cos(alpha_z) sin(alpha_z) 0; -sin(alpha_z) cos(alpha_z) 0; 0 0 1];

mat_rot_R_C = Rx_R_C * Ry_R_C * Rz_R_C
mat_rot_C_R = mat_rot_R_C'

%Matrice de translation
mat_trans_C_R = [Ocx_R; Ocy_R; Ocz_R];

% Matrice de passage Cam -> Robot
mat_Pss_C_R = [mat_rot_C_R mat_trans_C_R; zeros(1, 3) ones(1, 1)]

%Matrice de passage Robot -> Cam
mat_Pss_R_C = inv(mat_Pss_C_R)

% Matrice de passage Cam -> Image
K_C_I = [f 0 u0; 0 f v0; 0 0 1];

% Matrice de passage Image -> Cam
K_I_C = inv(K_C_I)

% Vecteur du pixel dans image
Px_I=[u; v; 1];

% Vecteur origine_cam-pixel dans le repère cam
V44_Px_C = [(K_I_C * Px_I); ones(1,1)]
% Vecteur origine_robot-pixel dans repère robot
V_Px_R = mat_Pss_C_R * ( V44_Px_C)

% Coordonnées de l'orgine cam dans le robot
Oc_R = [Ocx_R; Ocy_R; Ocz_R; 1];

% Vecteur Pix-cam dans le repère robot
V44_PxC_R = V_Px_R - Oc_R;







V_Px2_R = Oc_R + (zM - Oc_R(3))/V44_PxC_R(3)*V44_PxC_R

% Matrice de passage Cam -> Image
K2_C_I = [f2 0 u20; 0 f2 v20; 0 0 1];

% Matrice de passage Image -> Cam
K2_I_C = inv(K2_C_I)

mat_Pss_R_C2 = [1 0 0 0; 0 -1 0 174; 0 0 -1 zM+1; 0 0 0 1];
mat_Pss_C2_R = inv(mat_Pss_R_C2);

V_Px2_C2 = mat_Pss_R_C2*V_Px2_R

V_Px2_I = K2_C_I * V_Px2_C2(1:3)


syms a;
syms b;

V_PxTLC_R = mat_Pss_C_R*[K_I_C*[a; b; 1]; 1];
V44_PxTLCC_R = V_PxTLC_R - Oc_R;
V_Px2TLC_R = Oc_R + (zM - Oc_R(3))/V44_PxTLCC_R(3)*V44_PxTLCC_R;
V_Px2TLC_C2 = mat_Pss_R_C2*V_Px2TLC_R;
V_Px2TLC_I = K2_C_I*V_Px2TLC_C2(1:3)

%[e,f]=solve('c=(335*((10*a)/5213 - 3195/5213))/(2^(1/2)/2 + (2^(1/2)*((10*b)/5213 - 2395/5213))/2) + 538','d=905 - (335*(2^(1/2)/2 - (2^(1/2)*((10*b)/5213 - 2395/5213))/2))/(2^(1/2)/2 + (2^(1/2)*((10*b)/5213 - 2395/5213))/2)','a','b')

V_PxBLC_R = mat_Pss_C_R*[K_I_C*[0; 479; 1]; 1];
V44_PxBLCC_R = V_PxBLC_R - Oc_R;
V_Px2BLC_R = Oc_R + (zM - Oc_R(3))/V44_PxBLCC_R(3)*V44_PxBLCC_R;
V_Px2BLC_C2 = mat_Pss_R_C2*V_Px2BLC_R;
V_Px2BLC_I = K2_C_I*V_Px2BLC_C2(1:3)



I = imread('vlcsnap-2014-02-28-06h51m32s99.png');
figure;imshow(I);title('image brute non redressée');

J = uint8(zeros(781,538*2,3));
% 
% for j=1:.2:479
%     for i=1:.7:639
%         V_Px_R = mat_Pss_C_R*[K_I_C*[i; j; 1]; 1];
%         V44_PxC_R = V_Px_R - Oc_R;
%         V_Px2_R = Oc_R + (zM - Oc_R(3))/V44_PxC_R(3)*V44_PxC_R;
%         V_Px2_C2 = mat_Pss_R_C2*V_Px2_R;
%         V_Px2_I = K2_C_I*V_Px2_C2(1:3);
%         
%         
%         
%         u = round(V_Px2_I(1));
%         v = round(V_Px2_I(2));
%         
%         J(v,u,:) = I(round(j),round(i),:);
%     end
% end

for j=1:781
    for i=1:538*2
        u = round((3195*j - 5213*2^(1/2)*i + 2804594*2^(1/2) - 3961800)/(10*(j - 1240)));
        v = round(-(1409*j - 805)/(5*j - 6200));
        
        
        if u>0 && u<641 && v>0 && v<481
            J(round(j),round(i),:) = I(v,u,:);
        end
    end
end

figure;imshow(J);title('image brute');

K=rgb2hsv(J);
figure;imshow(K);title('image en hsv');
%lll=K(:,:,1)>120/180;

L=K(:,:,1)>120/180 & K(:,:,1)<10/180;
figure;imagesc(L);
