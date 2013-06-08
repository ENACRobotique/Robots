function [ u_D1, u_D2, u_D3 ] = u_distances_aux_balises(D1, D2, D3)
    global r epsilon omega u_delta_t_distance u_omega;

    % calcul incertitude distance en fonction de la distance
    alpha = 2*asin(r./D1) + epsilon;    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)
    u_D1 = abs(-D1*omega.*sqrt(D1.^2 - r^2)/(2 * r))*u_delta_t_distance + abs(-D1.*delta_t.*sqrt(D1.^2 - r^2)/(2 * r))*u_omega;

    alpha = 2*asin(r./D2) + epsilon;    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)
    u_D2 = abs(-D2*omega.*sqrt(D2.^2 - r^2)/(2 * r))*u_delta_t_distance + abs(-D2.*delta_t.*sqrt(D2.^2 - r^2)/(2 * r))*u_omega;

    alpha = 2*asin(r./D3) + epsilon;    % angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / omega;    % temps correspondant à l'angle alpha ci-dessus (s)
    u_D3 = abs(-D3*omega.*sqrt(D3.^2 - r^2)/(2 * r))*u_delta_t_distance + abs(-D3.*delta_t.*sqrt(D3.^2 - r^2)/(2 * r))*u_omega;
end
