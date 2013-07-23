%% paramètres terrain
global B1 B2 B3 D12 D23 D31 E;

% position balises
B1 = [-0.04; -0.04];    % (m)
B2 = [-0.04; 2.04]; % (m)
B3 = [3.04; 1]; % (m)

% distances inter-balises
D12 = norm(B2 - B1);    % (m)
D23 = norm(B3 - B2);    % (m)
D31 = norm(B1 - B3);    % (m)

% dimension zone de déplacement
E = [3; 2];   % (m)

%% paramètres tourelle
global omega r epsilon u_delta_t_distance u_delta_t_angle u_omega;

omega = 20*2*pi;    % vitesse de rotation du rotor (rad/s)
r = 25e-3; % rayon du cercle sur lequel sont les deux lasers (m)
epsilon = 0.5*pi/180;  % erreur de parallélisme des lasers (rad)

% paramètres d'incertitude
u_delta_t_distance = 8e-6;  % (s)
u_delta_t_angle = 16e-6;  % (s)
u_omega = omega^2/(2*pi)*4e-6;  % (rad/s)
