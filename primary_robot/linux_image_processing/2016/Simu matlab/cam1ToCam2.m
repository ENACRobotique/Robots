clear all
close all
clc
% _______ Data _______
% Cam1 intrinsic parameters
f1 = 516.3;
w1 = 640;
h1 = 480;
% Cam1 extrinsic parameters
x1 = 0;
y1 = 12.7;
z1 = 26.7;
RxC1_R_deg = 226; % 226
% Cam2 extrinsic parameters
RxC2_R_deg = 180;
% Robot extrinsic parameters
xr = 50;
yr = 30;
zr = 0;
Rz_R_deg = 0;

% Display
sFrame = 10; %cm
deepCam1 = 40; %cm

% Table points
wT = 300/3; hT = 200/1.5; % 300; 200
c0 = [0 0 0];
c1 = [wT 0 0];
c2 = [wT hT 0];
c3 = [0 hT 0];
% _____________________
theta_deg = RxC1_R_deg - 180;
RxC1_R = RxC1_R_deg/180*pi;
RxC2_R = RxC2_R_deg/180*pi;

Rz_R = Rz_R_deg/180*pi;
aperAngle1 = [2*atan(w1/(2*f1));
              2*atan(h1/(2*f1))];
aperAngle1_deg = [aperAngle1(1)/pi*180; 
                  aperAngle1(2)/pi*180]

cornPg = [c0' c1' c2' c3'];

ah_h = aperAngle1(1)/2;
av_h = aperAngle1(2)/2;
theta = theta_deg/180*pi;

% Matrixes I C
K1_C2I = [f1 0 w1/2-1;
          0 f1 h1/2-1;
          0 0  1]
K1_I2C = inv(K1_C2I)

% Transition matrixes
T_c1ToR = [1 0           0          x1;
           0 cos(-RxC1_R) sin(-RxC1_R) y1;
           0 -sin(-RxC1_R) cos(-RxC1_R) z1;
           0  0           0         1];
T_c2ToR = [1 0           0          x1;
           0 cos(RxC2_R) sin(RxC2_R) y1;
           0 -sin(RxC2_R) cos(RxC2_R) z1;
           0  0           0         1];
T_RToPg = [cos(Rz_R)  -sin(Rz_R) 0 xr;
           sin(Rz_R)  cos(Rz_R) 0 yr;
           0            0           1 zr;
           0            0           0 1];

T_c1To_Pg = T_RToPg*T_c1ToR;
T_c2To_Pg = T_RToPg*T_c2ToR;
T_c1Toc2 = inv(T_c2ToR)*T_c1ToR;


% Points 
px = [sFrame 0 0];
py = [0 sFrame 0];
pz = [0 0 sFrame];

ptOR_R = [0 0 0];
ptOR_Pg = T_RToPg*[ptOR_R 1]';

ptOC_C = [0 0 0];
ptOC_R = T_c1ToR*[ptOC_C 1]';
ptOC_Pg = T_c1To_Pg*[ptOC_C 1]';


% Compute h2 the heigh of the useful part of the projected image
f2 = f1*cos(theta)
h2b = f2*tan(theta + av_h)
h2s = f2*tan(theta - av_h);
h2 = h2b - h2s

% Compute w2b the width of the useful part of the projected image
d2 = sqrt(f2^2 + h2b^2);
d1 = sqrt(f1^2 + (h1/2)^2);
w2b = w1*d2/d1
aperAngle2 = [2*atan(w2b/(2*f2)); 2*atan(h2b/f2)];
aperAngle2_deg = aperAngle2./pi*180
          

% Drawing
figure
hold on

% Drawing pg
fill3(cornPg(1, :), cornPg(2, :), cornPg(3, :), 'b');

% Drawing frame of the robot
px_Pg = T_RToPg*[px 1]';
py_Pg = T_RToPg*[py 1]';
pz_Pg = T_RToPg*[pz 1]';
drawFrame(ptOR_Pg(1:3, :), px_Pg(1:3, :), py_Pg(1:3, :), pz_Pg(1:3, :), ...
    'Robot');

% Drawing cam1
plot3(ptOC_Pg(1), ptOC_Pg(2), ptOC_Pg(3), 'ro');  % Origin
ptC = [0 0 deepCam1*2]; % z axis
pt_R = T_c1ToR*[ptC 1]';
pt_Pg = T_RToPg*pt_R;
line = [ptOC_Pg(1:3, :)'; pt_Pg(1:3, :)'];
plot3(line(:,1), line(:,2), line(:,3),'r-');
px_c1_Pg = T_c1To_Pg*[px 1]';
py_c1_Pg = T_c1To_Pg*[py 1]';
pz_c1_Pg = T_c1To_Pg*[pz 1]';
drawFrame(ptOC_Pg(1:3, :), px_c1_Pg(1:3, :), py_c1_Pg(1:3, :), pz_c1_Pg(1:3, :), ...
    'Cam1');
drawFoV(T_c1To_Pg, f1, [w1, h1], [0 0 1], [0 0 0], 'b');
corners1_C1 = drawFocalPlane(T_c1To_Pg, f1, [w1, h1], [0 0 1], [0 0 f1/100], 'b');

% Drawing cam2
px_c2_Pg = T_c2To_Pg*[px 1]';
py_c2_Pg = T_c2To_Pg*[py 1]';
pz_c2_Pg = T_c2To_Pg*[pz 1]';
drawFrame(ptOC_Pg(1:3, :), px_c2_Pg(1:3, :), py_c2_Pg(1:3, :), pz_c2_Pg(1:3, :), ...
    'Cam2');
drawFoV(T_c2To_Pg, f2, [w2b h2b*2], [0 0 1], [0 0 0], 'r');
corners2_C2 = drawFocalPlane(T_c2To_Pg, f2, [w2b h2b*2], [0 0 1], [0 0 f2/100], 'r');

% Draw the useful part of the image in the focal plane of cam2
cornersUseFocPlane = ones(4,4);
cornersUseFocPlane_Pg = ones(4,4);
cornersUseFocPlane(1,:) = T_c1Toc2*[corners1_C1(1,:) 1]';
cornersUseFocPlane(2,:) = T_c1Toc2*[corners1_C1(4,:) 1]';
cornersUseFocPlane(1, 1:3);
ptTest = T_c2To_Pg*cornersUseFocPlane;
[cornersUseFocPlane(1, 1:3) check] = plane_line_intersect([0 0 1]', [0 0 f2/100]', [0 0 0]', cornersUseFocPlane(1, 1:3)');
[cornersUseFocPlane(2, 1:3) check] = plane_line_intersect([0 0 1]', [0 0 f2/100]', [0 0 0]', cornersUseFocPlane(2, 1:3)');
plane_line_intersect([0 0 1]', [0 0 f2/100]', [0 0 0]', cornersUseFocPlane(2, 1:3)');
cornersUseFocPlane(3,:) = [corners2_C2(3,:) 1]';
cornersUseFocPlane(4,:) = [corners2_C2(2,:) 1]';

for i=1:4
   cornersUseFocPlane_Pg(i,:) = T_c2To_Pg*cornersUseFocPlane(i,:)';
end
fill3(cornersUseFocPlane_Pg(:, 1), cornersUseFocPlane_Pg(:, 2), cornersUseFocPlane_Pg(:, 3), 'k');

% Drawing pixels
K2_C2I = [f2 0 round(w2b/2)-1;
          0 f2 round(h2b/1)-1;
          0 0  1];
K2_I2C = inv(K2_C2I)

'--------I1->I2----------'
p_I1 = [640 480]
p_C1 = K1_I2C*[p_I1 1]'
p_C2 = T_c1Toc2*[p_C1;1]
p_I2 = round(K2_C2I*(p_C2(1:3, 1)./p_C2(3)))
p_R = T_c2ToR*p_C2
p_Pg = T_RToPg*p_R
plot3(p_Pg(1), p_Pg(2), p_Pg(3), 'rx');

'--------I2->I1-----------'
p_I2 = [832 899]
p_C2 = K2_I2C*[p_I2 1]'
p_C1 = inv(T_c1Toc2)*[p_C2; 1]
P_I1 = round(K1_C2I*(p_C1(1:3,1)./p_C1(3)))
p_R = T_c1ToR*p_C1

% Drawing test
% plot3(ptTest(1), ptTest(2), ptTest(3), 'ro');

grid on
xlabel('x'); ylabel('y'); zlabel('z')
axis equal
alpha(0.3)
z = linspace(0,35);
hold off


% Test to straigthen up the image
% I = imread('../Captures/1cube.jpg');
I = imread('/home/yoyo/Robots/primary_robot/linux_image_processing/2016/Captures/1cube.jpg');
figure;imshow(I);title('raw image');

size(I)
j_max = h2
i_max = w2b

J = uint8(zeros(size(I)));

Jidx = zeros(i_max, j_max, 2);


matK2_I2C = eye(4,4)
matK2_I2C(1:3,1:3) = K2_I2C

matK1_C2I = eye(4,4)
matK1_C2I(1:3,1:3) = K1_C2I

mat_I2ToI1 = matK1_C2I*inv(T_c1Toc2)*matK2_I2C
for j=1:j_max
    j
    for i=1:i_max
        P_I1 = mat_I2ToI1*[i j 1 1]';
        P_I1 = round(P_I1./P_I1(3));
        u = P_I1(1);
        v = P_I1(2);
        
        if u>0 && u<w1 && v>0 && v<h1
            Jidx(i,j,1) = v;
            Jidx(i,j,2) = u;
%             J(j,i,:) = I(v,u,:);
        else
            Jidx(i,j,1) = -1;
            Jidx(i,j,2) = -1;
        end
    end
end

for j=1:j_max
    j0=j
    for i=1:i_max
        if Jidx(i,j,1) >0 && Jidx(i,j,2) >0
            J(j,i,:) = I(Jidx(i,j,1), Jidx(i,j,2),:);
        end
    end
end
'end'
figure;imshow(J);title('processed image');
