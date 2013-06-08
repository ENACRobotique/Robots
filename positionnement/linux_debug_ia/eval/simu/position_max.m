function [ XF, YF, z ] = position_max(d, D1, D2, D3, A12, A23, A31)
    %% calculs liés à la densification du plan
    % d: densité de mesures par mètre (m^-1)

    global E;

    % nombre de mesures par axe
    n = d*E;  % (index)
    x = E(1)*( 1/(n(1)+1):1/(n(1)+1):1-1/(n(1)+1) ); % (m)
    y = E(2)*( 1-1/(n(2)+1):-1/(n(2)+1):1/(n(2)+1) )'; % (m)

    % positions à calculer
    xx = ones(n(2), 1)*x;    % (m)
    yy = y*ones(1, n(1));  % (m)

    % distances par rapport aux 3 balises
    [d1, d2, d3] = distances_aux_balises(xx, yy);    % (m)

    if nargin==7
        % angles par rapport aux 3 balises
        [a12, a23, a31] = angles_entre_balises(d1, d2, d3);    % (rad)
    end

    %% calculs relatifs aux données
    % calcul distances
    [u_D1, u_D2, u_D3] = u_distances_aux_balises(D1, D2, D3);  % (m)

    mul = 0.5;
    s_D1 = mul*u_D1; s_D2 = mul*u_D2; s_D3 = mul*u_D3;

    if nargin==7
        % calcul angles
        [u_A12, u_A23, u_A31] = u_angles_entre_balises(A12, A23, A31);    % (rad)

        mul = 0.5;
        s_A12 = mul*u_A12; s_A23 = mul*u_A23; s_A31 = mul*u_A31;
    end

    % surface d'erreur
    % que les distances
    % z = exp(- (d1 - D1).^2./s_D1^2 - (d2 - D2).^2./s_D2^2 - (d3 - D3).^2./s_D3^2);
    z = -((d1 - D1).^2./s_D1^2 + (d2 - D2).^2./s_D2^2 + (d3 - D3).^2./s_D3^2);

    if nargin==7
        % on rajoute les angles aux distances
        % z = z*exp(- (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2);
        z = z - (a12 - A12).^2./s_A12^2 - (a23 - A23).^2./s_A23^2 - (a31 - A31).^2./s_A31^2;
    end

    % quelques stats
    [ZF, IF] = max(z(:));
    [IF, JF] = ind2sub(size(z), IF);    % position max
    XF = x(JF);
    YF = y(IF);

    if nargout ~= 3
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
    end
end
