#include <math.h>
#include "params.h"

#include "perception.h"

void simu_perception(sPt_t *x, sPerception *p) {
    if(!x || !p)
        return;

    // distances
    p->d1 = sqrt(SQR(x->x - glob_params.B1.x) + SQR(x->y - glob_params.B1.y));
    p->d2 = sqrt(SQR(x->x - glob_params.B2.x) + SQR(x->y - glob_params.B2.y));
    p->d3 = sqrt(SQR(x->x - glob_params.B3.x) + SQR(x->y - glob_params.B3.y));

    // angles
    p->a12 = acos((SQR(p->d1) + SQR(p->d2) - SQR(glob_params.D12))/(2*p->d1*p->d2));   // (rad)
    p->a23 = acos((SQR(p->d2) + SQR(p->d3) - SQR(glob_params.D23))/(2*p->d2*p->d3));   // (rad)
    p->a31 = acos((SQR(p->d3) + SQR(p->d1) - SQR(glob_params.D31))/(2*p->d3*p->d1));   // (rad)
}

void estim_incertitude(sPerception *p) {
    float alpha, delta_t;

    if(!p)
        return;

    // incertitudes sur distances
    alpha = 2*asin(glob_params.r/p->d1) + glob_params.epsilon;    // angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / glob_params.omega;    // temps correspondant à l'angle alpha ci-dessus (s)
    p->u_d1 = fabs(-p->d1*glob_params.omega*sqrt(SQR(p->d1) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_delta_t_distance + fabs(-p->d1*delta_t*sqrt(SQR(p->d1) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_omega;

    alpha = 2*asin(glob_params.r/p->d2) + glob_params.epsilon;    // angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / glob_params.omega;    // temps correspondant à l'angle alpha ci-dessus (s)
    p->u_d2 = fabs(-p->d2*glob_params.omega*sqrt(SQR(p->d2) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_delta_t_distance + fabs(-p->d2*delta_t*sqrt(SQR(p->d2) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_omega;

    alpha = 2*asin(glob_params.r/p->d3) + glob_params.epsilon;    // angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / glob_params.omega;    // temps correspondant à l'angle alpha ci-dessus (s)
    p->u_d3 = fabs(-p->d3*glob_params.omega*sqrt(SQR(p->d3) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_delta_t_distance + fabs(-p->d3*delta_t*sqrt(SQR(p->d3) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_omega;

    // incertitudes sur angles
    delta_t = p->a12/glob_params.omega;
    p->u_a12 = fabs(glob_params.omega)*glob_params.u_delta_t_angle + fabs(delta_t)*glob_params.u_omega;

    delta_t = p->a23/glob_params.omega;
    p->u_a23 = fabs(glob_params.omega)*glob_params.u_delta_t_angle + fabs(delta_t)*glob_params.u_omega;

    delta_t = p->a31/glob_params.omega;
    p->u_a31 = fabs(glob_params.omega)*glob_params.u_delta_t_angle + fabs(delta_t)*glob_params.u_omega;
}

void bruite_perception(sPerception *p) {
    if(!p)
        return;

    p->d1 += uni_rand(-p->u_d1, p->u_d1);
    p->d2 += uni_rand(-p->u_d2, p->u_d2);
    p->d3 += uni_rand(-p->u_d3, p->u_d3);
    p->a12 += uni_rand(-p->u_a12, p->u_a12);
    p->a23 += uni_rand(-p->u_a23, p->u_a23);
    p->a31 += uni_rand(-p->u_a31, p->u_a31);
}

