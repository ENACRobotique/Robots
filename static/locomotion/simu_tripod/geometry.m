% theoretical geometric data
v1 = [13.458; -4.289]; % (cm)
v2 = [13.458; -4.289]; % (cm)
v3 = [13.458; -4.289]; % (cm)

% computation model, geometric data (L#, theta#, phi#)
L1 = norm(v1) % (cm)
L2 = norm(v2) % (cm)
L3 = norm(v3) % (cm)

phi1 = 30*pi/180; % (rad)
phi2 = 150*pi/180; % (rad)
phi3 = 270*pi/180; % (rad)

theta1 = phi1 + atan2(v1(2), v1(1)); % (rad)
theta2 = phi2 + atan2(v2(2), v2(1)); % (rad)
theta3 = phi3 + atan2(v3(2), v3(1)); % (rad)

% compute translation for each pod
OP1_rob = L1*[cos(theta1); sin(theta1); 0]; % (cm)
OP2_rob = L2*[cos(theta2); sin(theta2); 0]; % (cm)
OP3_rob = L3*[cos(theta3); sin(theta3); 0]; % (cm)

% compute rotation for each pod (rotation matrices based on angle between the 2 reference frames)
R_rob2p1 = [cos(phi1) sin(phi1) 0; -sin(phi1) cos(phi1) 0; 0 0 1]; % rob2p1
R_rob2p2 = [cos(phi2) sin(phi2) 0; -sin(phi2) cos(phi2) 0; 0 0 1]; % rob2p2
R_rob2p3 = [cos(phi3) sin(phi3) 0; -sin(phi3) cos(phi3) 0; 0 0 1]; % rob2p3

% transformation matrices for each pod (not used here)
P_rob2p1 = [R_rob2p1 -R_rob2p1*OP1_rob; 0 0 0 1];
P_rob2p2 = [R_rob2p2 -R_rob2p2*OP2_rob; 0 0 0 1];
P_rob2p3 = [R_rob2p3 -R_rob2p3*OP3_rob; 0 0 0 1];

% calculate setpoint for each pod for linear speed only
v_rob = 10*[cos(10*pi/180); sin(10*pi/180); 0]; % (cm/s)
v_v_p1 = dot(R_rob2p1*v_rob, [0;1;0]) % (cm/s)
v_v_p2 = dot(R_rob2p2*v_rob, [0;1;0]) % (cm/s)
v_v_p3 = dot(R_rob2p3*v_rob, [0;1;0]) % (cm/s)

% calculate setpoint for each pod for angular speed only
omega_rob = [0;0;5*pi/180]; % (rad/s)
v_o_p1 = dot(R_rob2p1*cross(omega_rob, OP1_rob), [0;1;0]) % (cm/s)
v_o_p2 = dot(R_rob2p2*cross(omega_rob, OP2_rob), [0;1;0]) % (cm/s)
v_o_p3 = dot(R_rob2p3*cross(omega_rob, OP3_rob), [0;1;0]) % (cm/s)

% get speed setpoint for each pod
v_p1 = v_v_p1 + v_o_p1 % (cm/s)
v_p2 = v_v_p2 + v_o_p2 % (cm/s)
v_p3 = v_v_p3 + v_o_p3 % (cm/s)

% formula extracted from operations above (_v_p# must be equal to previously computed v_p#)
_v_p1 = -v_rob(1)*sin(phi1) + v_rob(2)*cos(phi1) + L1*omega_rob(3)*cos(phi1 - theta1) % (cm/s)
_v_p2 = -v_rob(1)*sin(phi2) + v_rob(2)*cos(phi2) + L2*omega_rob(3)*cos(phi2 - theta2) % (cm/s)
_v_p3 = -v_rob(1)*sin(phi3) + v_rob(2)*cos(phi3) + L3*omega_rob(3)*cos(phi3 - theta3) % (cm/s)

% build single static matrix from formula above
M_rob2pods = [ -sin(phi1) cos(phi1) L1*cos(phi1 - theta1);
               -sin(phi2) cos(phi2) L2*cos(phi2 - theta2);
               -sin(phi3) cos(phi3) L3*cos(phi3 - theta3)]
M_pods2rob = inv(M_rob2pods)

% verify results
vo_rob = v_rob + omega_rob;
v_pods = M_rob2pods*vo_rob
_vo_rob = M_pods2rob*v_pods
disp(["    => ", num2str(atan2(_vo_rob(2), _vo_rob(1))*180/pi), "°"])
disp(["    => ", num2str(norm(_vo_rob(1:2))), "cm/s"])
disp(["    => ", num2str(_vo_rob(3)*180/pi), "°/s"])

% =================================================
% trying to get geometric data back from the matrix

% with this kind of matrix, it's impossible to get the true L1,L2,L3 and theta1,theta2,theta3...
% we get an equivalent system with the three wheel axis intersecting in one point
A = M_rob2pods;
_sphi1 = -A(1, 1);
_cphi1 =  A(1, 2);
_phi1 = atan2(_sphi1, _cphi1);
disp(["    => phi1=", num2str(_phi1*180/pi), "°"])
_sphi2 = -A(2, 1);
_cphi2 =  A(2, 2);
_phi2 = atan2(_sphi2, _cphi2);
disp(["    => phi2=", num2str(_phi2*180/pi), "°"])
_sphi3 = -A(3, 1);
_cphi3 =  A(3, 2);
_phi3 = atan2(_sphi3, _cphi3);
disp(["    => phi3=", num2str(_phi3*180/pi), "°"])
_L1 = A(1, 3) % (cm)
_L2 = A(2, 3) % (cm)
_L3 = A(3, 3) % (cm)

% if you assume that theta1,theta2,theta3 are perfectly respected, you can get L1,L2,L3
_theta1 =  30*pi/180; % (rad)
_theta2 = 150*pi/180; % (rad)
_theta3 = 270*pi/180; % (rad)
__L1 = A(1, 3)/cos(_phi1 - _theta1)
__L2 = A(2, 3)/cos(_phi2 - _theta2)
__L3 = A(3, 3)/cos(_phi3 - _theta3)

