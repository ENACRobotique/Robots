%% derniere modif sur Matlab R2013a Linux
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
R_ROBOT = 15;

% obstacles
obs = [
10  , 170 , 0, R_ROBOT;
%foyer
0.  , 0.  , R_ROBOT 25;
300., 0.  , R_ROBOT 25;
150., 95. , R_ROBOT 15;
% torche mobile
90. , 90. , R_ROBOT 8;
210., 90. , R_ROBOT 8;
%torche fixe
0.  , 120., R_ROBOT 5;
130., 0.  , R_ROBOT 5;
170., 0.  , R_ROBOT 5;
300., 120., R_ROBOT 5;
% arbres
0.  , 70. , R_ROBOT 15;
70. , 0.  , R_ROBOT 15;
230., 0.  , R_ROBOT 15;
300., 70. , R_ROBOT 15;
%feux
40. , 90. , R_ROBOT 7;
90. , 40. , R_ROBOT 7;
90. , 140., R_ROBOT 7;
210., 40. , R_ROBOT 7;
210., 140., R_ROBOT 7;
260., 110., R_ROBOT 7;
0.  , 120., R_ROBOT 7;
130., 0.  , R_ROBOT 7;
170., 0.  , R_ROBOT 7;
300., 120., R_ROBOT 7;
90. , 90. , R_ROBOT 7;
90. , 90. , R_ROBOT 7;
90. , 90. , R_ROBOT 7;
210., 90. , R_ROBOT 7;
210., 90. , R_ROBOT 7;
210., 90. , R_ROBOT 7;
% bac fruit
55., 185. , R_ROBOT 15;
75., 185. , R_ROBOT 15;
95., 185. , R_ROBOT 15;
42., 172. , R_ROBOT 4;
108., 172., R_ROBOT 4;
205., 185., R_ROBOT 15;
225., 185., R_ROBOT 15;
245., 185., R_ROBOT 15;
192., 172., R_ROBOT 4;
258., 172., R_ROBOT 4;
% destination
260 ,  80 , 0 0;

140.572540 82.072395 0 0;
];


% path
pth = [
%10.000000, 170.000000;
%78.789078, 123.437538;% 90.000000, 140.000000, 20.00, 12.920848, 83.066238
%91.019501, 120.026001;
%148.572693, 122.963600;%, 150.000000, 95.000000, 28.00, 2.855813, 57.628113
%151.427307, 122.963600;
%208.980499, 120.026001;%, 210.000000, 140.000000, 20.00, 5.330941, 57.628113
%214.277695, 120.462822;
%255.722305, 129.537170;%, 260.000000, 110.000000, 20.00, 46.198891, 42.426399
%277.320496, 100.000000;
%260.000000, 70.000000;%, 260.000000, 70.000000, 0.00, 0.000000, 34.641010

%10.000000 170.000000;
%78.789078 123.437538;
%91.019501 120.026001; 148.572693 122.963600;
%155.553650 122.443710; 214.165237 110.582779;
%224.785202 104.913010; 260.000000 70.000000;

10.000000 170.000000; 77.231644 122.084389;
92.690521 118.165138; 146.331100 124.774811;
155.950333 124.403969; 214.561920 112.543045;
224.198807 108.094025; 260.000000 80.000000;

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


for i=2:size(pth, 1)
	% objets
	plot(pth(i, 1), pth(i, 2), '+k')
	text(pth(i, 1), pth(i, 2), strcat([' ', num2str(i-1)]));
    
    plot([pth(i, 1);pth(i-1, 1)], [pth(i, 2);pth(i-1, 2)]);
end

% limites
%plot([xmin; xmax; xmax; xmin; xmin], [ymin; ymin; ymax; ymax; ymin], 'r');
