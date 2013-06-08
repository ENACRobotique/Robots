#include "optim.h"

unsigned int nb_cri=0;

float critere(sPt_t *x, sPerception *p) {
    sPerception simu_p;

    if(!x || !p)
        return 0.;

    nb_cri++;

    simu_perception(x, &simu_p);

    return (
        SQR(simu_p.d1 - p->d1)/SQR(p->u_d1) +
        SQR(simu_p.d2 - p->d2)/SQR(p->u_d2) +
        SQR(simu_p.d3 - p->d3)/SQR(p->u_d3) +

        SQR(simu_p.a12 - p->a12)/SQR(p->u_a12) +
        SQR(simu_p.a23 - p->a23)/SQR(p->u_a23) +
        SQR(simu_p.a31 - p->a31)/SQR(p->u_a31)
    );
}

