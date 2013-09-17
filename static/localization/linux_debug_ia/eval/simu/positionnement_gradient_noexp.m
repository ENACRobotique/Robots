clear all;
close all;
clc;

%% paramètres terrain
parametres_systeme;

% point pour lequel on applique la méthode du gradient
X0 = 1.2;
Y0 = 1;

[D1, D2, D3, u_D1, u_D2, u_D3] = distances_aux_balises(X0, Y0);
% TODO: implémenter cet algo en tenant compte des angles

mul = 0.5;
s_D1 = mul*u_D1; s_D2 = mul*u_D2; s_D3 = mul*u_D3;

% on bruite un peu les distances en fonction de l'incertitude
D1 = D1 + u_D1/2*randn; D2 = D2 + u_D2/2*randn; D3 = D3 + u_D3/2*randn;

% on calcule le meilleur positionnement possible avec la méthode du max du critère (pour comparer)
[XF, YF] = position_max(1000, D1, D2, D3);

%% calcul itératif
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
alpha = 2e-5;

for k=1:100
    [d1, d2, d3] = distances_aux_balises(x, y);  % (m)

    % critère à minimiser
    z = (d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2;
    
    if z <= z_old
        alpha = 1.2 * alpha;
        x_old = x;
        y_old = y;
        z_old = z;
    else
        alpha = 0.7 * alpha;
        x = x_old;
        y = y_old;
        z = z_old;
        [d1, d2, d3] = distances_aux_balises(x, y);  % (m)
    end
    
    G = 2*[ (d1 - D1)*(x - B1(1))/(d1 * s_D1^2) + (d2 - D2)*(x - B2(1))/(d2 * s_D2^2) + (d3 - D3)*(x - B3(1))/(d3 * s_D3^2);
            (d1 - D1)*(y - B1(2))/(d1 * s_D1^2) + (d2 - D2)*(y - B2(2))/(d2 * s_D2^2) + (d3 - D3)*(y - B3(2))/(d3 * s_D3^2) ];

    XL = [XL, x];
    YL = [YL, y];
    ZL = [ZL, z];
    AL = [AL, alpha];
    GL = [GL, G];

    if norm(G) < 2
        break;
    end

    p = [x; y] - alpha*G;
    x = p(1);
    y = p(2);
end

disp(['Position finale : (', num2str(x), ', ', num2str(y), ', ', num2str(z), '), atteinte en ', num2str(k), ' itérations']);
disp(['Erreur de position finale : ', num2str(sqrt((X0 - x).^2 + (Y0 - y).^2)), ' m']);
disp(['Erreur de position (/ max bruité) : ', num2str(sqrt((XF - x).^2 + (YF - y).^2)), ' m']);

hold on;
plot3(XL, YL, ones(size(XL)), 'x-');

figure, plot(XL, YL, '+-');
title('Chemin du point dans le plan');
hold on;
plot(XL(1), YL(1), 'xr', XL(end), YL(end), 'or', X0, Y0, '+g', XF, YF, '+g');
text(XL(1), YL(1), ' \leftarrow départ');
text(XL(end), YL(end), ' \leftarrow arrivée');
text(X0, Y0, ' \leftarrow solution');
text(XF, YF, ' \leftarrow solution bruitée');

figure, plot(ZL, '+-');
title('Evolution du critère Z');
xlabel('numéro itération');
ylabel('Z');

figure, plot(AL, '+-');
title('Evolution du pas');
xlabel('numéro itération');
ylabel('pas');

figure, plot(sqrt(GL(1, :).^2 + GL(2, :).^2), '+-');
title('Evolution de la norme du gradient');
xlabel('numéro itération');
ylabel('||\nabla||');
