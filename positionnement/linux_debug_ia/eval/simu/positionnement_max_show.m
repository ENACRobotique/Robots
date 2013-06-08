clear all;
close all;
clc;

%% description terrain
parametres_systeme;

% densité de mesures par mètre
d = 300;    % (m^-1)

% nombre de mesures par axe
n = d*E;  % (index)
x = E(1)*( 1/(n(1)+1):1/(n(1)+1):1-1/(n(1)+1) ); % (m)
y = E(2)*( 1-1/(n(2)+1):-1/(n(2)+1):1/(n(2)+1) )'; % (m)

% positions à calculer
xx = ones(n(2), 1)*x;    % (m)
yy = y*ones(1, n(1));  % (m)

% distances par rapport aux 3 balises
[d1, d2, d3] = distances_aux_balises(xx, yy);    % (m)

% angles sous lesquels on voit les 3 balises
[a12, a23, a31] = angles_entre_balises(d1, d2, d3);    % (rad)

%% on calcule pour un point
% point pour lequel on calcule la surface d'incertitude
X0 = 2.9;
Y0 = 1.1;

% calcul distances
[D1, D2, D3, u_D1, u_D2, u_D3] = distances_aux_balises(X0, Y0);  % (m)

mul = 100;
s_D1 = mul*u_D1; s_D2 = mul*u_D2; s_D3 = mul*u_D3;

% calcul angles
[A12, A23, A31, u_A12, u_A23, u_A31] = angles_entre_balises(D1, D2, D3);    % (rad)

mul = 100;
s_A12 = mul*u_A12; s_A23 = mul*u_A23; s_A31 = mul*u_A31;

% on bruite les mesures en fonction de leur incertitude pour modéliser le bruit
D1 = D1 + u_D1/2*randn; D2 = D2 + u_D2/2*randn; D3 = D3 + u_D3/2*randn;
A12 = A12 + u_A12/2*randn; A23 = A23 + u_A23/2*randn; A31 = A31 + u_A31/2*randn;

% surface d'erreur
% que les distances
% z = exp(- (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2);
% z = -((d1 - D1).^2./sigma1^2 + (d2 - D2).^2./sigma2^2 + (d3 - D3).^2./sigma3^2);
% z = exp(- (d3 - D3).^2./s_D3^2);

% que les angles
% z = exp(- (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2);
% z = - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2;
% z = exp( - (a12 - A12).^2./s_A12^2);
% z = exp( - (a23 - A23).^2./s_A23^2);
% z = exp( - (a31 - A31).^2./s_A31^2);

% distances + angles
z = exp(- (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2 - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2);
% z = - (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2 - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2;

% quelques stats
[ZF, IF] = max(z(:));
[IF, JF] = ind2sub(size(z), IF);    % position max
XF = x(JF);
YF = y(IF);

disp(['Le max vaut ', num2str(ZF), ' et est en (', num2str(XF), ', ', num2str(YF), ') m']);
disp(['Erreur de position : ', num2str(sqrt((X0 - XF)^2 + (Y0 - YF)^2)), ' m']);

%% affichage
% affichage résultat
if d<=200   % affichage de tout ...
    x_ = x;
    y_ = y;
    z_ = z;
else % ... sinon juste autour du max
    JF_ = max(JF-100,1):min(JF+100,n(1));
    IF_ = max(IF-100,1):min(IF+100,n(2));
    z_ = z(IF_, JF_);
    x_ = x(JF_);
    y_ = y(IF_);
end
figure, surf(x_, y_, z_);
% axis([x_(1), x_(end), y_(end), y_(1)]);
xlabel('x (m)');
ylabel('y (m)');
zlabel('densité proba (à un facteur près)');
text(XF, YF, ZF*1.01, '\leftarrow max');
text(X0, Y0, ZF*1.01, '\leftarrow max théo');

% affichage gradient
% if d<=50   % affichage de tout ...
%     x_ = x;
%     y_ = y;
%     z_ = z;
% else % ... sinon juste autour du max
%     JF_ = max(JF-25,1):min(JF+25,n(1));
%     IF_ = max(IF-25,1):min(IF+25,n(2));
%     z_ = z(IF_, JF_);
%     x_ = x(JF_);
%     y_ = y(IF_);
% end
% [px, py] = gradient(z_, x_, y_);
% figure, contour(x_, y_ , z_), hold on, quiver(x_, y_, px, py), hold off
% figure, surf(x_, y_, sqrt(px.^2 + py.^2));
