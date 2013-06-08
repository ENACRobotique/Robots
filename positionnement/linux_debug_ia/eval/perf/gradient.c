#include <stdio.h>
#include <math.h>
#include "optim.h"

#include "gradient.h"

#define GR_GAIN_ADAPT
#define GR_DEBUG
#ifdef GR_DEBUG
#   define GR_WARN_JUMP
#endif

void gradient(sPt_t *x0, sPerception *p) {
    sPt_t tmp;
    float z, step, alpha, nG;
#if defined(GR_GAIN_ADAPT)
    sPt_t x_old;
    float z_old;
#endif
#if defined(GR_WARN_JUMP)
    float z_prev;
#endif
    sVec_t G;
    int i;

    // let's go
    step = 1e-3;
    alpha = 1e-7;

    for(i=0; i<1000; i++) {
        z = critere(x0, p);

#ifdef GR_GAIN_ADAPT
        if(!i || z <= z_old) {
#   ifdef GR_DEBUG
            printf("###ok    (%f <= %f)### ", z, z_old);
#   endif
            alpha *= 1.2;
            x_old = *x0;
            z_old = z;
        }
        else {
#   ifdef GR_DEBUG
            printf("###failed (%f > %f)### ", z, z_old);
#   endif
            alpha *= 0.5;
            *x0 = x_old;
            z = z_old;
        }
#endif

        // compute gradient
        tmp.x = x0->x + step; tmp.y = x0->y;   G.x = (critere(&tmp, p) - z)/step;
        tmp.x = x0->x; tmp.y = x0->y + step;   G.y = (critere(&tmp, p) - z)/step;
        nG = sqrt(SQR(G.x) + SQR(G.y));

#ifdef GR_DEBUG
        printf("IT%03u: en %.3fm, %.3fm: %03.0f, %f, dir %.2f°", i, x0->x, x0->y, z, nG, 180.*atan2(-G.y, -G.x)/M_PI);
#   ifdef GR_GAIN_ADAPT
        printf(", alpha %e", alpha);
#   endif
#   ifdef GR_WARN_JUMP
        if(i && z > z_prev)
            printf(", /!\\ bad step /!\\");
        z_prev = z;
#   endif
        printf("\n");
#endif

        if(z < Z_CRIT)//nG < 2e3)
            break;

        x0->x -= alpha*G.x;
        x0->y -= alpha*G.y;
    }

#ifdef GR_DEBUG
    printf("%u itérations\n", i);
#endif
}

