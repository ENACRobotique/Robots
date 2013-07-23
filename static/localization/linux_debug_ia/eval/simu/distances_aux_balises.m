function [D1, D2, D3, u_D1, u_D2, u_D3] = distances_aux_balises(X, Y)
    global B1 B2 B3;

    % distances par rapport aux 3 balises : seules ces données doivent être utilisées pour le reste des calculs
    D1 = sqrt((X - B1(1)).^2 + (Y - B1(2)).^2);   % (m)
    D2 = sqrt((X - B2(1)).^2 + (Y - B2(2)).^2);   % (m)
    D3 = sqrt((X - B3(1)).^2 + (Y - B3(2)).^2);   % (m)

    if nargout == 6
        [u_D1, u_D2, u_D3] = u_distances_aux_balises(D1, D2, D3);

        % we avoid to have a huge difference between values
        % U = [u_D1, u_D2, u_D3];
        % [M1, I1] = min(U);
        % U(I1) = inf;
        % [M2, I2] = min(U);
        % if M2 > 10*M1
        %     U(I1) = M2/10;
        %     disp(['u_D', num2str(I1), ' raised to ', num2str(M2/10)]);
        % else
        %     U(I1) = M1;
        % end
        % u_D1 = U(1);
        % u_D2 = U(2);
        % u_D3 = U(3);
    end
end
