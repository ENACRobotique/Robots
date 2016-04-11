
% Init
clear all
clc
close all 

% Macro
    rad2deg = 180./pi;
    deg2rad = pi/180.;
    
% Data
    l2_sq = power(0.004, 2) + power(0.102, 2);
    l2 = sqrt(l2_sq);
    l3_sq = power(0.039, 2) + power(0.08, 2);
    l3 = sqrt(l3_sq);
    l45_x = 0.042 + 0.03;
    l45_z = 0.005;
    theta2_off = -atan2(-0.004, 0.102);
    theta3_off = atan2(0.035, 0.08);
    theta4_off = atan2(-0.005, 0.03 + 0.042);
    theta4_off_deg = theta4_off*rad2deg
    x2_b = 0.0365;
    z2_b = 0.1 + 0.03 + 0.0358;

    thetamin = zeros(4,1);
    thetamax = zeros(4,1);
    thetamin(1) = -1.57;
    thetamax(1) = 1.57;
    thetamin(2) = -2.356;
    thetamax(2) = 0.785;
    thetamin(3) = -3.14;
    thetamax(3) = 0;
    thetamin(4) = -2.356;
    thetamax(4) = 0.785;
    
    x = 0;
    y = 0.15;
    z = 0.1;
    beta = 0;
% End Data


% End macro

    r = sqrt(power(x,2) + power(y,2));
    x2t_2 = r - x2_b;
    z2t_2 = z - z2_b;

    % Compute the origin O4 of the link 3 in the reference frame of link2
    v = [-l45_x*sin(beta) + l45_z*cos(beta) , 0.0, l45_x*cos(beta) + l45_z*sin(beta)];
    pt_2 = [x2t_2, 0, z2t_2]
    O4_2 = pt_2 + v
    xO4_2 = O4_2(1);
    zO4_2 = O4_2(3);

    r24_sq = power(xO4_2, 2) + power(zO4_2, 2);
    r24 = sqrt(r24_sq)
    nAlpha2 = (l3_sq - l2_sq - r24_sq)/(-2*l2*r24);
    arg2 = min(max(nAlpha2, -1), 1);
    alpha2 = acos(arg2);
    alpha2_deg = alpha2*rad2deg 
    gamma2 = atan2(zO4_2, xO4_2);
    gamma2_deg = gamma2*rad2deg

    % Compute the value of "_J1"
    disp('____ J1 _____')
    theta2 = -pi/2. + alpha2 + gamma2;
    theta2_deg = theta2*rad2deg
    J1 = -theta2 + theta2_off;
    J1_deg = J1*rad2deg
    
    % Compute the value of theta2
    disp('____ J2 _____')
    nArg3 = (r24_sq - l2_sq - l3_sq)/(-2*l2*l3)
    arg3 = min(max(nArg3, -1.), 1.)
    theta3 = acos(arg3);
    theta3_deg = theta3*rad2deg
    J2 = -theta3 + theta3_off;
    J2_deg = J2*rad2deg
    
    % Compute the vlaue of theta3
    disp('____ J3 _____')
    theta4 = -theta2 - beta - theta3 + theta3_off;
    theta4_deg = theta4*rad2deg
    J3 = theta4 + theta4_off;
    J3_deg = J3*rad2deg

    %% Data from moveit
    disp('____ Comparisons _____')
    O4_b_mv = [-4.6922e-06; 0.076413; 0.18666]
    O4_2_est = O4_b_mv - [0; x2_b; z2_b]
    O4_2
    
    pt_b_mv = [-6.7679e-06; 0.098644; 0.11799]
    pt_2_est = pt_b_mv - [0; x2_b; z2_b]
    pt_2