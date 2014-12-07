clear all;
close all;
clc; % clear console

%% Calcul de f2 en supposant une position caméra simple (tangage de 45° uniquement)
f2_mm = 67; % (mm)
sz=[640;480]; % (px)
theta = 2*atan(sz(2)/2/521.3); % (rad)
L = f2_mm*tan(theta/2+pi/4); % (mm)
l = f2_mm*tan(pi/4-theta/2); % (mm)
f2_px = 781*f2_mm/(L-l); % (px)
v0 = L*f2_px/f2_mm; % (px)
f2 = f2_px;
clear L l theta sz v0 f2_px
% keep f2

%% Paramètres extrinsèques Cam1
% Angles de rotation du repère cam
    alpha_x = 225*pi/180; %radian
    alpha_y = 0;
    alpha_z = 0;
% Coordonnées origine cam dans repère robot
    Ocx_R = 0;
    Ocy_R = 107;
    Ocz_R = 267;

% Matrices de rotations
Rx_R_C = [1 0 0; 0 cos(alpha_x) sin(alpha_x); 0 -sin(alpha_x) cos(alpha_x)];
Ry_R_C = [cos(alpha_y) 0 -sin(alpha_y); 0 1 0; sin(alpha_y) 0 cos(alpha_y)];
Rz_R_C = [cos(alpha_z) sin(alpha_z) 0; -sin(alpha_z) cos(alpha_z) 0; 0 0 1];

mat_rot_R_C = Rx_R_C * Ry_R_C * Rz_R_C;
mat_rot_C_R = mat_rot_R_C';

% Matrice de translation
mat_trans_C_R = [Ocx_R; Ocy_R; Ocz_R];

% Matrice de passage Cam -> Robot
mat_Pss_C_R = [mat_rot_C_R mat_trans_C_R; 0 0 0 1];

% Matrice de passage Robot -> Cam
mat_Pss_R_C = inv(mat_Pss_C_R);

% Coordonn�es de l'origine cam dans le robot
Oc_R = mat_Pss_C_R * [0; 0; 0; 1];

%% Paramètres intrinsèques Cam1
% distance focale
    f = 521.3; % (px)
% Centre de l'image en pixel
    u0 = (0+639)/2;
    v0 = (0+479)/2;

% Matrice de passage Cam -> Image
K_C_I = [f 0 u0; 0 f v0; 0 0 1];

% Matrice de passage Image -> Cam
K_I_C = inv(K_C_I);

%% Paramètres extrinsèques Cam2
mat_Pss_C2_R = [1  0  0 Oc_R(1);
                0 -1  0 Oc_R(2);
                0  0 -1 Oc_R(3);
                0  0  0       1];
mat_Pss_R_C2 = inv(mat_Pss_C2_R);

%% Paramètres intrinsèques Cam2
% distance focale
    f2 = 334.2; % (px)
% Centre de l'image en pixel
    u20 = 536;
    v20 = 903;

% Matrice de passage Cam2 -> Image
K_C2_I = [f2 0 u20; 0 f2 v20; 0 0 1];

% Matrice de passage Image -> Cam2
K_I_C2 = inv(K_C2_I);

%% Exemple pour un pixel
disp(['For a pixel:']);

Px_I=[u0; v0; 1]

% Vecteur origine_cam-pixel dans le rep�re cam
V44_Px_C = [(K_I_C * Px_I); 1];
% Vecteur origine_robot-pixel dans rep�re robot
V_Px_R = mat_Pss_C_R * ( V44_Px_C);

% Vecteur Pix-cam dans le repere robot
V44_PxC_R = V_Px_R - Oc_R;

% Intersection de ce vecteur avec un plan horizontal
V_Px2_R = Oc_R + -Oc_R(3)/V44_PxC_R(3)*V44_PxC_R;

V_Px2_C2 = mat_Pss_R_C2*V_Px2_R;

V_Px2_I = K_C2_I * V_Px2_C2(1:3);
V_Px2_I = V_Px2_I/V_Px2_I(3)

%% Example with symbolic variables
% syms u v;
% 
% disp(['Symbolic:']);
% Px_I=[u; v; 1]
% V44_PxC_R = mat_Pss_C_R * [(K_I_C * Px_I); 1] - Oc_R;
% V_Px2_R = Oc_R + -Oc_R(3)/V44_PxC_R(3)*V44_PxC_R;
% V_Px2_C2 = mat_Pss_R_C2*V_Px2_R;
% V_Px2_I = K_C2_I * V_Px2_C2(1:3);
% V_Px2_I = V_Px2_I/V_Px2_I(3)
% 
% syms u2 v2;
% [u1,v1] = solve(V_Px2_I(1:2) == [u2;v2], u, v)
% clear u2 v2;

%% Example for top left hand corner
disp(['Top left hand corner:']);
Px_I=[0; 0; 1];
V44_PxC_R = mat_Pss_C_R * [(K_I_C * Px_I); 1] - Oc_R;
V_Px2_R = Oc_R + -Oc_R(3)/V44_PxC_R(3)*V44_PxC_R;
V_Px2_C2 = mat_Pss_R_C2*V_Px2_R;
V_Px2_I = K_C2_I * V_Px2_C2(1:3);
V_Px2_I = V_Px2_I/V_Px2_I(3)

%% Example for bottom left hand corner
disp(['Bottom left hand corner:']);
Px_I=[0; 479; 1];
V44_PxC_R = mat_Pss_C_R * [(K_I_C * Px_I); 1] - Oc_R;
V_Px2_R = Oc_R + -Oc_R(3)/V44_PxC_R(3)*V44_PxC_R;
V_Px2_C2 = mat_Pss_R_C2*V_Px2_R;
V_Px2_I = K_C2_I * V_Px2_C2(1:3);
V_Px2_I = V_Px2_I/V_Px2_I(3)

%% Redressement image
I = imread('vlcsnap-2014-02-28-06h51m32s99.png');
figure;imshow(I);title('image brute non redressée');

J = uint8(zeros(781,536*2,3));

for j=1:781
    for i=1:536*2
        u2 = i-1;
        v2 = j-1;
        u = round((3195*v2 - 5213*2^(1/2)*u2 + 2794168*2^(1/2) - 3952854)/(10*v2 - 12372))+1;
        v = round(-(7045*v2 - 5151)/(25*v2 - 30930))+1;

        if u>0 && u<641 && v>0 && v<481
            J(round(j),round(i),:) = I(v,u,:);
        end
    end
end

figure;imshow(J);title('image brute');

%% Conversion HSV
% K=rgb2hsv(J);
% figure;imshow(K);title('image en hsv');
% %lll=K(:,:,1)>120/180;
% 
%% Filtrage HSV
% L=(K(:,:,1)>150.4/180 | K(:,:,1)<10/180) & K(:,:,2) > 0.45;
% figure;imagesc(L);

%% De la camera 2 vers la table
disp(['Cam2 vers table']);
syms u2 v2;
Px2_I=[u2; v2; 1];
V44_PxC2_R = mat_Pss_C2_R * [(K_I_C2 * Px2_I); 1] - Oc_R;
V_Px3_R = Oc_R + (0-Oc_R(3))/V44_PxC2_R(3)*V44_PxC2_R
