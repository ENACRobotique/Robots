%% derniere modif sur Octave 0.10.1
clear all;
close all;
clc;

%% ___ donnees ___

% limites
xmin = 20;
ymin = 20;
xmax = 300-xmin;
ymax = 200-ymin;

% rayon robot
rr = 20;

% obstacles
obs = [
% depart
30 100 0 rr;
% gateau
150 200 rr 50;
% verres bleus
90 55 rr 4;
90 105 rr 4;
105 80 rr 4;
135 80 rr 4;
120 55 rr 4;
120 105 rr 4;
% verres rouges
300-90 55 rr 4;
300-90 105 rr 4;
300-105 80 rr 4;
300-135 80 rr 4;
300-120 55 rr 4;
300-120 105 rr 4;
% arrivee
250 50 0 0
];

%% ___ affichage ___

x = 0:300;
theta = 0:pi/50:2*pi;
figure; hold on; axis([0 300 0 200], 'equal');

obs(:,3) = obs(:, 3) + obs(:, 4);

for i=1:size(obs, 1)
	% objets
	plot(obs(i, 1), obs(i, 2), '+b')
	text(obs(i, 1), obs(i, 2), strcat([' ', num2str(i-1)]));
	if obs(i, 4) > 0.01
		plot(obs(i, 4)*cos(theta)+obs(i, 1), obs(i, 4)*sin(theta)+obs(i, 2), 'b');
	end

	% zones interdites objets
	if obs(i, 3) > obs(i, 4) + 0.01
		plot(obs(i, 3)*cos(theta)+obs(i, 1), obs(i, 3)*sin(theta)+obs(i, 2), 'r');
	end
end

% limites
plot([xmin; xmax; xmax; xmin; xmin], [ymin; ymin; ymax; ymax; ymin], 'r');
