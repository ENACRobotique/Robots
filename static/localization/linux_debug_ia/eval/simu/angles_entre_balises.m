function [A12, A23, A31, u_A12, u_A23, u_A31] = angles_entre_balises(d1, d2, d3)
    global D12 D23 D31;

    % angles
    A12 = acos((d1.^2 + d2.^2 - D12^2)./(2*d1.*d2));   % (rad)
    A23 = acos((d2.^2 + d3.^2 - D23^2)./(2*d2.*d3));   % (rad)
    A31 = acos((d3.^2 + d1.^2 - D31^2)./(2*d3.*d1));   % (rad)
    
    if nargout == 6
        [u_A12, u_A23, u_A31] = u_angles_entre_balises(A12, A23, A31);
        
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
    
    if imag(A12)
        A12 = real(A12);
        u_A12 = inf;
    end
    
    if imag(A23)
        A23 = real(A23);
        u_A23 = inf;
    end
    
    if imag(A31)
        A31 = real(A31);
        u_A31 = inf;
    end
end
