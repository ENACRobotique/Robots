#include <stdio.h>
#include <math.h>
#include "optim.h"

#include "neldermead.h"

//#define NM_DEBUG

int neldermead(sPt_t *x0, float range, sPerception *p) {
    sPt_t xb, xr, xe, xc, x[3];
    int i, j, i_max, i_MAX, i_MIN;
    float z[3], z_xr, z_xe, z_xc, alpha=1., gamma=2., rho=-0.5, sigma=0.5;

    for(j=0; j<3; j++) {
        x[j].x = x0->x + range*cos(120.*(float)j*M_PI/180.);
        x[j].y = x0->y + range*sin(120.*(float)j*M_PI/180.);
        z[j] = critere(&x[j], p);

#ifdef NM_DEBUG
        printf("x[%u]=(%.2f, %.2f, %.2f)\n", j, x[j].x, x[j].y, z[j]);
#endif

        if(z[j] < Z_CRIT) {
#ifdef NM_DEBUG
            printf("=> ret x[%u]\n", j);
#endif
            *x0 = x[j];
            return 0;
        }
    }

    for(i=0; i<20; i++) {
// tri
        i_MAX = 0;
        i_MIN = 0;
        for(j=1; j<3; j++) {
            if(z[j] < z[i_MIN])
                i_MIN = j;

            if(z[j] > z[i_MAX])
                i_MAX = j;
        }
        i_max = 3 - i_MIN - i_MAX;
#ifdef NM_DEBUG
        printf("   i_MIN = %u i_max = %u i_MAX = %u\n", i_MIN, i_max, i_MAX);
#endif

// barycentre
        xb.x = 0;
        xb.y = 0;
        for(j=0; j<3; j++) {
            if(j != i_MAX) {
                xb.x += x[j].x/2.;
                xb.y += x[j].y/2.;
            }
        }
#ifdef NM_DEBUG
        printf("   xb=(%.2f, %.2f)\n", xb.x, xb.y);
#endif

// calcul point réflechi
        xr.x = xb.x + alpha*(xb.x - x[i_MAX].x);
        xr.y = xb.y + alpha*(xb.y - x[i_MAX].y);
        z_xr = critere(&xr, p);
#ifdef NM_DEBUG
        printf("   xr=(%.2f, %.2f, %.2f)\n", xr.x, xr.y, z_xr);
#endif
        if(z_xr < Z_CRIT) {
#ifdef NM_DEBUG
            printf("=> ret xr\n");
#endif
            *x0 = xr;
            return 0;
        }

// reflexion
        if(z[i_MIN] <= z_xr && z_xr < z[i_max]) {
#ifdef NM_DEBUG
        printf("=> x[%u] = xr\n", i_MAX);
#endif
            x[i_MAX] = xr;
            z[i_MAX] = z_xr;
            continue;
        }

// expansion
        if(z_xr < z[i_MIN]) {
            xe.x = xb.x + gamma*(xb.x - x[i_MAX].x);
            xe.y = xb.y + gamma*(xb.y - x[i_MAX].y);
            z_xe = critere(&xe, p);
#ifdef NM_DEBUG
            printf("   xe=(%.2f, %.2f, %.2f)\n", xe.x, xe.y, z_xe);
#endif
            if(z_xe < Z_CRIT) {
#ifdef NM_DEBUG
                printf("=> ret xe\n");
#endif
                *x0 = xe;
                return 0;
            }

            if(z_xe < z_xr) {
#ifdef NM_DEBUG
                printf("=> x[%u] = xe\n", i_MAX);
#endif
                x[i_MAX] = xe;
                z[i_MAX] = z_xe;
            }
            else {
#ifdef NM_DEBUG
                printf("=> x[%u] = xr\n", i_MAX);
#endif
                x[i_MAX] = xr;
                z[i_MAX] = z_xr;
            }
            continue;
        }

// contraction
        xc.x = xb.x + rho*(xb.x - x[i_MAX].x);
        xc.y = xb.y + rho*(xb.y - x[i_MAX].y);
        z_xc = critere(&xc, p);
#ifdef NM_DEBUG
        printf("xc=(%.2f, %.2f, %.2f)\n", xc.x, xc.y, z_xc);
#endif
        if(z_xc < Z_CRIT) {
#ifdef NM_DEBUG
            printf("=> ret xc\n");
#endif
            *x0 = xc;
            return 0;
        }

        if(z_xc < z_xr) {
#ifdef NM_DEBUG
            printf("=> x[%u] = xc\n", i_MAX);
#endif
            x[i_MAX] = xc;
            z[i_MAX] = z_xc;
            continue;
        }

// reduction
        for(j=0; j<3; j++) {
            if(j == i_MIN)
                continue;

            x[j].x = x[i_MIN].x + sigma*(x[j].x - x[i_MIN].x);
            x[j].y = x[i_MIN].y + sigma*(x[j].y - x[i_MIN].y);
            z[j] = critere(&x[j], p);
#ifdef NM_DEBUG
            printf("=> x[%u] = (%.2f, %.2f, %.2f)\n", j, x[j].x, x[j].y, z[j]);
#endif
            if(z[j] < Z_CRIT) {
#ifdef NM_DEBUG
                printf("=> ret x[%u]\n", j);
#endif
                *x0 = x[j];
                return 0;
            }
        }
    }

// too much iterations
#ifdef NM_DEBUG
    printf("=> no convergence:");
#endif

    i_MIN = 0;
    for(j=0; j<3; j++) {
        z[j] = critere(&x[j], p);

#ifdef NM_DEBUG
        printf(" z[%i]=%.2f", j, z[j]);
#endif

        if(z[j] < z[i_MIN])
            i_MIN = j;
    }
#ifdef NM_DEBUG
    printf("\n=> ret x[%u]\n", i_MIN);
#endif
    *x0 = x[i_MIN];
    return 1;
}

