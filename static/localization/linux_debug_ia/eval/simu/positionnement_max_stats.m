clear all;
close all;
clc;

%% description terrain
parametres_systeme;

% densité de mesures par mètre
d = 1000;    % (m^-1)

% nombre de mesures par axe
n = d*E;  % (index)
x = E(1)*( 1/(n(1)+1):1/(n(1)+1):1-1/(n(1)+1) ); % (m)
y = E(2)*( 1-1/(n(2)+1):-1/(n(2)+1):1/(n(2)+1) )'; % (m)

% positions à calculer
xx = ones(n(2), 1)*x;    % (m)
yy = y*ones(1, n(1));  % (m)

% distances par rapport aux 3 balises
[d1, d2, d3] = distances_aux_balises(xx, yy);    % (m)

% angles par rapport aux 3 balises
[a12, a23, a31] = angles_entre_balises(d1, d2, d3);    % (rad)

%% on calcule pour N points
N = 150;
err = zeros(N, 1);

% point pour lequel on calcule la surface d'incertitude
X0 = 2.9;
Y0 = 1.1;

% calcul distances
[D1_0, D2_0, D3_0, u_D1, u_D2, u_D3] = distances_aux_balises(X0, Y0);  % (m)

mul = 0.5;
s_D1 = mul*u_D1; s_D2 = mul*u_D2; s_D3 = mul*u_D3;

% calcul angles
[A12_0, A23_0, A31_0, u_A12, u_A23, u_A31] = angles_entre_balises(D1_0, D2_0, D3_0);    % (rad)

mul = 0.5;
s_A12 = mul*u_A12; s_A23 = mul*u_A23; s_A31 = mul*u_A31;

for k=1:N
    disp(['Calcul #', num2str(k)]);

    % on bruite les mesures en fonction de leur incertitude pour modéliser le bruit
    D1 = D1_0 + u_D1/2*randn; D2 = D2_0 + u_D2/2*randn; D3 = D3_0 + u_D3/2*randn;
    A12 = A12_0 + u_A12/2*randn; A23 = A23_0 + u_A23/2*randn; A31 = A31_0 + u_A31/2*randn;

    % surface d'erreur
    % que les distances
%     z = exp(- (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2);
%     z = -((d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2);

    % que les angles
%     z = exp(- (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2);
%     z = - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2;

    % distances + angles
    z = exp(- (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2 - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2);
%     z = - (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2 - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2;

    % quelques stats
    [ZF, IF] = max(z(:));
    [IF, JF] = ind2sub(size(z), IF);    % position max
    XF = x(JF);
    YF = y(IF);

%     disp(['Le max vaut ', num2str(ZF), ' et est en (', num2str(XF), ', ', num2str(YF), ') m']);
    err(k) = sqrt((X0 - XF).^2 + (Y0 - YF).^2);

    disp(['Erreur de position : ', num2str(err(k)), ' m']);
end

disp(['err : min ', num2str(min(err)), ', mean ', num2str(mean(err)), ', median ', num2str(median(err)), ', max ', num2str(max(err)), ', std ', num2str(std(err))]);
