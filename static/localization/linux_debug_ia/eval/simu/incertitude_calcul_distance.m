close all;
clear all;
clc;

% calcul/affichage de l'incertitude sur le calcul de la distance d'une balise à la tourelle.

% parameters
omega = 20*2*pi;    % vitesse de rotation du rotor (rad/s)
r = 25; % rayon du cercle sur lequel sont les deux lasers (mm)
D = 100:10:sqrt(2000^2+1000^2); % distance du centre du rotor au récepteur (mm)
epsilon = [0; 0.5]*pi/180; % erreur d'alignement des lasers : on le fait pour plusieurs erreurs pour comparer (rad)

%% value
alpha = 2*asin((r*ones(size(epsilon))*ones(size(D)))./(ones(size(epsilon))*D)) + epsilon*ones(size(D));    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)

figure, plot(delta_t, D);
legend(strcat(ones(size(epsilon))*'\epsilon=', num2str(epsilon*180/pi), ones(size(epsilon))*'°'));
xlabel('\Deltat (s)');
ylabel('L (mm)');

%% uncertainty
% delta {L} sur delta {delta_t} : dépendance de L par rapport à delta_t
D_delta_t = -D*omega.*sqrt(D.^2 - r^2*ones(size(D)))./(2 * r);

figure, plot(D_delta_t, D);
legend('\forall\epsilon');  % ne dépend pas d'epsilon
xlabel('{\deltaL} sur {\delta\Deltat} (mm.s^{-1})');
ylabel('L (mm)');

% delta {L} sur delta {omega} : dépendance de L par rapport à omega
L_omega = -ones(size(epsilon))*D.*delta_t.*sqrt(ones(size(epsilon))*D.^2 - r^2*ones(size(epsilon))*ones(size(D)))./(2 * r);

figure, plot(L_omega, D);
legend(strcat(ones(size(epsilon))*'\epsilon=', num2str(epsilon*180/pi), ones(size(epsilon))*'°'));
xlabel('{\deltaL} sur {\delta\omega} (mm.s.rad^{-1})');
ylabel('L (mm)');
