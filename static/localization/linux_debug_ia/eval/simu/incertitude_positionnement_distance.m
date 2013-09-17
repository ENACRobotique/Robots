clear all;
close all;
clc;

% calcul/affichage de l'incertitude sur le calcul de positionnement en utilisant uniquement 2 distances sur 3 (la troisisème ne sert qu'à lever l'indétermination).

%% paramètres fixes (terrain)
% positions balises
B1 = [-0.04; -0.04];    % (m)
B2 = [-0.04; 2.04]; % (m)
B3 = [3.04; 1]; % (m)

% dimensions zone de déplacement
DIM = [3; 2];   % (m)

% distances inter-balises
D12 = norm(B2 - B1);    % (m)
D23 = norm(B3 - B2);    % (m)
D31 = norm(B1 - B3);    % (m)

% matrices de changement de repères
P12 = [B2(1)-B1(1), B1(2)-B2(2); B2(2)-B1(2), B2(1)-B1(1)]/D12;
P23 = [B3(1)-B2(1), B2(2)-B3(2); B3(2)-B2(2), B3(1)-B2(1)]/D23;
P31 = [B1(1)-B3(1), B3(2)-B1(2); B1(2)-B3(2), B1(1)-B3(1)]/D31;

% paramètres tourelle
omega = 20*2*pi;    % vitesse de rotation du rotor (rad/s)
r = 25e-3; % rayon du cercle sur lequel sont les deux lasers (m)
epsilon = 0*pi/180;  % erreur d'alignement des lasers (rad)

% paramètres d'incertitude
u_delta_t = 4e-6;  % (s)
u_omega = 0.05*2*pi;  % (rad/s)

u_seuil = 50e-3;    % (m)

%% init positions à calculer

% densité de mesures par mètre
d = 100;    % (m^-1)

% nombre de mesures par axe
N = d*DIM;  % (index)
X = DIM(1)/(N(1)+1):DIM(1)/(N(1)+1):DIM(1)-DIM(1)/(N(1)+1); % (m)
Y = (DIM(2)-DIM(2)/(N(2)+1):-DIM(2)/(N(2)+1):DIM(2)/(N(2)+1))'; % (m)

% positions à calculer
X0 = ones(N(2), 1)*X;    % (m)
Y0 = Y*ones(1, N(1));  % (m)

% distances par rapport aux 3 balises : seules ces données doivent être utilisées pour le reste des calculs
D1 = sqrt((X0 - B1(1)).^2 + (Y0 - B1(2)).^2);   % (m)
D2 = sqrt((X0 - B2(1)).^2 + (Y0 - B2(2)).^2);   % (m)
D3 = sqrt((X0 - B3(1)).^2 + (Y0 - B3(2)).^2);   % (m)

% calcul incertitude distance en fonction de la distance
alpha = 2*asin(r./D1) + epsilon;    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)
u_D1 = abs(-D1*omega.*sqrt(D1.^2 - r^2)/(2 * r))*u_delta_t + abs(-D1.*delta_t.*sqrt(D1.^2 - r^2)/(2 * r))*u_omega;

alpha = 2*asin(r./D2) + epsilon;    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)
u_D2 = abs(-D2*omega.*sqrt(D2.^2 - r^2)/(2 * r))*u_delta_t + abs(-D2.*delta_t.*sqrt(D2.^2 - r^2)/(2 * r))*u_omega;

alpha = 2*asin(r./D3) + epsilon;    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)
u_D3 = abs(-D3*omega.*sqrt(D3.^2 - r^2)/(2 * r))*u_delta_t + abs(-D3.*delta_t.*sqrt(D3.^2 - r^2)/(2 * r))*u_omega;

clear alpha delta_t;

% position d'une intersection dans le nouveau repère "balises"
U12 = D12/2 + (D1.^2 - D2.^2)/(2*D12);  % (m)
U23 = D23/2 + (D2.^2 - D3.^2)/(2*D23);  % (m)
U31 = D31/2 + (D3.^2 - D1.^2)/(2*D31);  % (m)

V12 = sqrt(D1.^2 - U12.^2); % (m)
V23 = sqrt(D2.^2 - U23.^2); % (m)
V31 = sqrt(D3.^2 - U31.^2); % (m)

% calcul incertitude positionnement dans ce repère
u_U12 = abs(D1/D12).*u_D1 + abs(-D2/D12).*u_D2;
u_U23 = abs(D2/D23).*u_D2 + abs(-D3/D23).*u_D3;
u_U31 = abs(D3/D31).*u_D3 + abs(-D1/D31).*u_D1;

u_V12 = abs(D1.*(1 - U12/D12)./sqrt(D1.^2 - U12.^2)).*u_D1 + abs(D2.*U12/D12./sqrt(D1.^2 - U12.^2)).*u_D2;
u_V23 = abs(D2.*(1 - U23/D23)./sqrt(D2.^2 - U23.^2)).*u_D2 + abs(D3.*U23/D23./sqrt(D2.^2 - U23.^2)).*u_D3;
u_V31 = abs(D3.*(1 - U31/D31)./sqrt(D3.^2 - U31.^2)).*u_D3 + abs(D1.*U31/D31./sqrt(D3.^2 - U31.^2)).*u_D1;

clear u_D1 u_D2 u_D3;

% on repasse dans le repère table
X112 = reshape((B1*ones(1, prod(N)) + P12*[U12(:)'; V12(:)'])', N(2), N(1), 2);
Y112 = X112(:, :, 2); X112 = X112(:, :, 1);
X212 = reshape((B1*ones(1, prod(N)) + P12*[U12(:)'; -V12(:)'])', N(2), N(1), 2);
Y212 = X212(:, :, 2); X212 = X212(:, :, 1);

clear U12 V12;

X123 = reshape((B2*ones(1, prod(N)) + P23*[U23(:)'; V23(:)'])', N(2), N(1), 2);
Y123 = X123(:, :, 2); X123 = X123(:, :, 1);
X223 = reshape((B2*ones(1, prod(N)) + P23*[U23(:)'; -V23(:)'])', N(2), N(1), 2);
Y223 = X223(:, :, 2); X223 = X223(:, :, 1);

clear U23 V23;

X131 = reshape((B3*ones(1, prod(N)) + P31*[U31(:)'; V31(:)'])', N(2), N(1), 2);
Y131 = X131(:, :, 2); X131 = X131(:, :, 1);
X231 = reshape((B3*ones(1, prod(N)) + P31*[U31(:)'; -V31(:)'])', N(2), N(1), 2);
Y231 = X231(:, :, 2); X231 = X231(:, :, 1);

clear U31 V31;

% on cherche à lever l'indétermination avec la troisième distance non utilisée (selon le cas)
M  = abs(sqrt((X112 - B3(1)).^2 + (Y112 - B3(2)).^2) - D3) < abs(sqrt((X212 - B3(1)).^2 + (Y212 - B3(2)).^2) - D3);
X12 = X212; X12(M) = X112(M);
Y12 = Y212; Y12(M) = Y112(M);

clear X112 Y112 X212 Y212;

M  = abs(sqrt((X123 - B1(1)).^2 + (Y123 - B1(2)).^2) - D1) < abs(sqrt((X223 - B1(1)).^2 + (Y223 - B1(2)).^2) - D1);
X23 = X223; X23(M) = X123(M);
Y23 = Y223; Y23(M) = Y123(M);

clear X123 Y123 X223 Y223;

M  = abs(sqrt((X131 - B2(1)).^2 + (Y131 - B2(2)).^2) - D2) < abs(sqrt((X231 - B2(1)).^2 + (Y231 - B2(2)).^2) - D2);
X31 = X231; X31(M) = X131(M);
Y31 = Y231; Y31(M) = Y131(M);

clear X131 Y131 X231 Y231;
clear M;

% vérif algo position
seuil = 1e-10;
if max(max(sqrt((X0 - X12).^2 + (Y0 - Y12).^2))) > seuil
    warning('method 12 failed (%d/%d > %e)', sum(sum(sqrt((X0 - X12).^2 + (Y0 - Y12).^2) > seuil)), prod(N), seuil);
end
if max(max(sqrt((X0 - X23).^2 + (Y0 - Y23).^2))) > seuil
    warning('method 23 failed (%d/%d > %e)', sum(sum(sqrt((X0 - X23).^2 + (Y0 - Y23).^2) > seuil)), prod(N), seuil);
end
if max(max(sqrt((X0 - X31).^2 + (Y0 - Y31).^2))) > seuil
    warning('method 31 failed (%d/%d > %e)', sum(sum(sqrt((X0 - X31).^2 + (Y0 - Y31).^2) > seuil)), prod(N), seuil);
end
clear seuil;

% calcul incertitude
u_XY12 = sqrt(u_U12.^2 + u_V12.^2);
u_XY23 = sqrt(u_U23.^2 + u_V23.^2);
u_XY31 = sqrt(u_U31.^2 + u_V31.^2);

clear u_U12 u_V12;
clear u_U23 u_V23;
clear u_U31 u_V31;

% show uncertainty of each method on 3 figures
% figure, surf(min(u_XY12, u_seuil));
% title('u_XY12');
% figure, surf(min(u_XY23, u_seuil));
% title('u_XY23');
% figure, surf(min(u_XY31, u_seuil));
% title('u_XY31');

% compute best uncertainty
u_XY = min(u_XY12, min(u_XY23, u_XY31));

% show best uncertainty
label = 0*(u_XY == u_XY12) + 1*(u_XY == u_XY23) + 2*(u_XY == u_XY31);
figure, surf(X, Y, u_XY, label);
hold on;
surf(X, Y, u_seuil*ones(size(u_XY)), 'EdgeColor', 'none', 'FaceAlpha', 0.4);
xlabel('x (m)');
ylabel('y (m)');
zlabel('EPU (m)');  % estimated position uncertainty
text(DIM(1), DIM(2), u_seuil, 'seuil CDC');

% some statistics on uncertainty
disp(['Min uncertainty: ', num2str(min(u_XY(:)))]);
disp(['Max uncertainty: ', num2str(max(u_XY(:)))]);
disp(['Mean uncertainty: ', num2str(mean(u_XY(:)))]);
disp(['Median uncertainty: ', num2str(median(u_XY(:)))]);
disp([num2str(100*sum(sum(u_XY > u_seuil))/prod(N)), '% values higher than ', num2str(u_seuil)]);
figure, hist(u_XY(:), prod(N)/300);
xlabel('EPU (m)');
