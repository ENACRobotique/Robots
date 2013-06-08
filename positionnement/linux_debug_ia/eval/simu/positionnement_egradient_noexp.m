clear all;
close all;
clc;

%% paramtres terrain
parametres_systeme;

% point pour lequel on applique la mthode du gradient
X0 = 1.5;
Y0 = 0.5;

% pos : med(grad)  1mm / med(grad)^-1  1cm / med(u_Di)
% 2.8;0.9 : 4e4 / 2.8e-6 / 0.18
% 0.2;0.2 : 2e4 / 4.0e-6 / 0.069
% 2.5;1.0 : 9e3 / 1.2e-5 / 0.15
% 0.5;0.5 : 6e3 / 1.6e-5 / 0.054
% 1.5;0.5 : 2e3 / 4.0e-5 / 0.054

[D1, D2, D3, u_D1, u_D2, u_D3] = distances_aux_balises(X0, Y0);
[A12, A23, A31, u_A12, u_A23, u_A31] = angles_entre_balises(D1, D2, D3);    % (rad)

mul = 0.5;
s_D1 = mul*u_D1; s_D2 = mul*u_D2; s_D3 = mul*u_D3;
mul = 0.5;
s_A12 = mul*u_A12; s_A23 = mul*u_A23; s_A31 = mul*u_A31;

% on bruite un peu les distances en fonction de l'incertitude
D1 = D1 + u_D1/2*randn; D2 = D2 + u_D2/2*randn; D3 = D3 + u_D3/2*randn;
A12 = A12 + u_A12/2*randn; A23 = A23 + u_A23/2*randn; A31 = A31 + u_A31/2*randn;

% on calcule le meilleur positionnement possible avec la mthode du max du critre (pour comparer)
[XF, YF] = position_max(1000, D1, D2, D3, A12, A23, A31);   % la surface affiche ne correspond pas au mme critre que la descente de gradient ci dessous
% XF = X0;
% YF = Y0;

%% calcul itratif
% position initiale
x = X0 + 0.05*randn;
y = Y0 + 0.05*randn;

x_old = x;
y_old = y;
z_old = inf;

XL = [];
YL = [];
ZL = [];
AL = [];
GL = [];
alpha = 1e-7;
step = 1e-6;

for k=1:100
    [d1, d2, d3] = distances_aux_balises(x, y);  % (m)
    [a12, a23, a31] = angles_entre_balises(d1, d2, d3); % (rad)

    % critre  minimiser
    z = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2 + (a12 - A12).^2./s_A12^2 + (a23 - A23).^2./s_A23^2 + (a31 - A31).^2./s_A31^2;
    
    if z <= z_old
        alpha = 1.2 * alpha;
        x_old = x;
        y_old = y;
        z_old = z;
    else
        alpha = 0.5 * alpha;
        x = x_old;
        y = y_old;
        z = z_old;
    end

    % calcul gradient
    [d1, d2, d3] = distances_aux_balises(x + step, y);  % (m)
    [a12, a23, a31] = angles_entre_balises(d1, d2, d3); % (rad)
    z_x = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2 + (a12 - A12).^2./s_A12^2 + (a23 - A23).^2./s_A23^2 + (a31 - A31).^2./s_A31^2;
    [d1, d2, d3] = distances_aux_balises(x, y + step);  % (m)
    [a12, a23, a31] = angles_entre_balises(d1, d2, d3); % (rad)
    z_y = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2 + (a12 - A12).^2./s_A12^2 + (a23 - A23).^2./s_A23^2 + (a31 - A31).^2./s_A31^2;
    
    G = [ z_x - z ; z_y - z ]/step;

    XL = [XL, x];
    YL = [YL, y];
    ZL = [ZL, z];
    AL = [AL, alpha];
    GL = [GL, G];

    if norm(G) < 2e3
        break;
    end

    p = [x; y] - alpha*G;
    x = p(1);
    y = p(2);
end

disp(['Position initiale : (', num2str(XL(1)), ', ', num2str(YL(1)), ', ', num2str(ZL(1)), ')']);
disp(['Erreur initiale : ', num2str(sqrt((X0 - XL(1)).^2 + (Y0 - YL(1)).^2)), ' m']);
disp(['Position finale : (', num2str(x), ', ', num2str(y), ', ', num2str(z), '), atteinte en ', num2str(k), ' itrations']);
disp(['Erreur finale : ', num2str(sqrt((X0 - x).^2 + (Y0 - y).^2)), ' m']);
disp(['Erreur finale (/ bruit) : ', num2str(sqrt((XF - x).^2 + (YF - y).^2)), ' m']);

hold on;
% affichage trajectoire par dessus la surface de proba
plot3(XL, YL, ones(size(XL)), 'o-');

% affichage du cercle unit centr sur la meilleure solution
theta = 0:pi/20:2*pi;
R = 0.001;
plot3(XF+R*cos(theta), YF+R*sin(theta), 2*ones(size(theta)), '-g');

N = 12;
R = 0.01;
Glim = zeros(N, 1);
Zlim = zeros(N, 1);
for k=1:N
    x = XL(end)+R*cos(k*2*pi/N);
    y = YL(end)+R*sin(k*2*pi/N);
    
    [d1, d2, d3] = distances_aux_balises(x, y);  % (m)
    [a12, a23, a31] = angles_entre_balises(d1, d2, d3); % (rad)
    z = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2 + (a12 - A12).^2./s_A12^2 + (a23 - A23).^2./s_A23^2 + (a31 - A31).^2./s_A31^2;

    Zlim(k) = z;
    
    [d1, d2, d3] = distances_aux_balises(x + step, y);  % (m)
    [a12, a23, a31] = angles_entre_balises(d1, d2, d3); % (rad)
    z_x = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2 + (a12 - A12).^2./s_A12^2 + (a23 - A23).^2./s_A23^2 + (a31 - A31).^2./s_A31^2;
    
    [d1, d2, d3] = distances_aux_balises(x, y + step);  % (m)
    [a12, a23, a31] = angles_entre_balises(d1, d2, d3); % (rad)
    z_y = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2 + (a12 - A12).^2./s_A12^2 + (a23 - A23).^2./s_A23^2 + (a31 - A31).^2./s_A31^2;
    
    G = [ z_x - z ; z_y - z ]/step;
    
    Glim(k) = norm(G);
end
disp(['Glim : min ', num2str(min(Glim)), ', mean ', num2str(mean(Glim)), ', median ', num2str(median(Glim)), ', max ', num2str(max(Glim)), ', std ', num2str(std(Glim))]);
disp(['Zlim : min ', num2str(min(Zlim)), ', mean ', num2str(mean(Zlim)), ', median ', num2str(median(Zlim)), ', max ', num2str(max(Zlim)), ', std ', num2str(std(Zlim))]);

% 1/median(Glim)
% median([u_D1, u_D2, u_D3])

figure, plot(XL, YL, '+-');
title('Trajectoire dans le plan');
hold on;
plot(XL(1), YL(1), 'xr', XL(end), YL(end), 'or', X0, Y0, '+g', XF, YF, '+g');
text(XL(1), YL(1), ' \leftarrow dpart');
text(XL(end), YL(end), ' \leftarrow arrive');
text(X0, Y0, ' \leftarrow solution');
text(XF, YF, ' \leftarrow solution bruite');
R = 0.001;
plot(XF+R*cos(theta), YF+R*sin(theta), '-g');

figure;
subplot(2, 2, 1), plot(sqrt((XF - XL).^2 + (YF - YL).^2), '+-');
title('Evolution de la distance  la solution');
xlabel('numro itration');
ylabel('distance  la solution');

subplot(2, 2, 2), plot(ZL, '+-');
title('Evolution du critre');
xlabel('numro itration');
ylabel('Z');

subplot(2, 2, 3), plot(AL, '+-');
title('Evolution du pas');
xlabel('numro itration');
ylabel('pas');

subplot(2, 2, 4), plot(sqrt(GL(1, :).^2 + GL(2, :).^2), '+-');
title('Evolution de la norme du gradient');
xlabel('numro itration');
ylabel('||\nablaZ||');
