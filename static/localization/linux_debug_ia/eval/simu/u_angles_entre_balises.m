function [ u_A12, u_A23, u_A31 ] = u_angles_entre_balises(A12, A23, A31)
    global omega u_delta_t_angle u_omega;

    delta_t = A12/omega;
    u_A12 = abs(omega)*u_delta_t_angle + abs(delta_t)*u_omega;

    delta_t = A23/omega;
    u_A23 = abs(omega)*u_delta_t_angle + abs(delta_t)*u_omega;

    delta_t = A31/omega;
    u_A31 = abs(omega)*u_delta_t_angle + abs(delta_t)*u_omega;
end
