#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

#define NELDERMEAD
//#define GRADIENT

//#define FIXED_SEED 1
//#define FIXED_INPUT_PROBLEM
#define STATS_MAP

#include "params.h"
#include "tools.h"
#include "perception.h"
#include "optim.h"
#ifdef NELDERMEAD
#   include "neldermead.h"
#endif
#ifdef GRADIENT
#   include "gradient.h"
#endif

int main(int argc, char *argv[]) {
    sPt_t x0 /* reference */, x /* start point */;
    sPerception p;
#if !defined(FIXED_INPUT_PROBLEM)
    float dir;
#endif
#ifdef STATS_MAP
    FILE *flog;
    int i;
    float m;
#else
    sPt_t tmp;
#endif

    // entry point
    init_globals();

#ifdef FIXED_SEED
    srand(FIXED_SEED);
#else
    srand(time(NULL) + (int)getpid());
#endif

    // builds the noisy perception data
#ifdef STATS_MAP
    flog = fopen("out.csv", "wb+");
    if(!flog)
        return 1;

    for(x0.x=0.05; x0.x<=2.95; x0.x+=0.05) {
        for(x0.y=0.05; x0.y<=1.95; x0.y+=0.05) {
            m = 0;
            nb_cri = 0;

            for(i=0; i<50; i++) {
                simu_perception(&x0, &p);
                estim_incertitude(&p);
                bruite_perception(&p);

                dir = uni_rand(-M_PI, M_PI);
                x.x = x0.x + 0.02*cos(dir);
                x.y = x0.y + 0.02*sin(dir);

                neldermead(&x, 0.05, &p);

                m += sqrt(SQR(x.y - x0.y) + SQR(x.x - x0.x));
            }

            fprintf(flog, "%.2f;%.2f;%.2f;%.4f\n", x0.x, x0.y, (float)nb_cri/(float)i, m/(float)i);
        }
    }

    fclose(flog);

    return 0;
#else   // STATS_MAP
#   ifdef FIXED_INPUT_PROBLEM
    x0.x = 1.5;
    x0.y = 0.5;

    simu_perception(&x0, &p);
    estim_incertitude(&p);

    p.d1 = 1.60459;
    p.d2 = 2.27835;
    p.d3 = 1.60727;
    p.a12 = 1.1214;
    p.a23 = 2.04323;
    p.a31 = 3.11656;

    x.x = 1.519;
    x.y = 0.48867;
#   else    // FIXED_INPUT_PROBLEM
    if(argc == 3) {
        printf("Using starting point from arguments:\n");
        x0.x = strtof(argv[1], NULL);
        x0.y = strtof(argv[2], NULL);
    }
    else {
        x0.x = 2.8;
        x0.y = 0.8;
    }
    printf("x0(%.2f, %.2f)\n", x0.x, x0.y);

    simu_perception(&x0, &p);
    estim_incertitude(&p);
    bruite_perception(&p);

//    printf("u_d1=%.3fm, u_d2=%.3fm, u_d3=%.3fm\n", p.u_d1, p.u_d2, p.u_d3);
//    printf("u_a12=%.2f°, u_a23=%.2f°, u_a31=%.2f°\n", p.u_a12*180./M_PI, p.u_a23*180./M_PI, p.u_a31*180./M_PI);

    // add some random bias to the start point
    dir = uni_rand(-M_PI, M_PI);
    x.x = x0.x + 0.02*cos(dir);
    x.y = x0.y + 0.02*sin(dir);
    printf("err dir=%.1f°\n", dir*180./M_PI);
#   endif   // FIXED_INPUT_PROBLEM

#   ifdef NELDERMEAD
    tmp = x;
    nb_cri = 0;

    neldermead(&tmp, 0.05, &p);

    printf("%u calculs du critère\n", nb_cri);
    printf("nm_res=(%.2f, %.2f) (erreur de %.6fm)\n", tmp.x, tmp.y, sqrt(SQR(tmp.y - x0.y) + SQR(tmp.x - x0.x)));
#   endif   // NELDERMEAD

#   ifdef GRADIENT
    tmp = x;
    nb_cri = 0;

    gradient(&tmp, &p);

    printf("%u calculs du critère\n", nb_cri);
    printf("gr_res=(%.2f, %.2f) (erreur de %.6fm)\n", tmp.x, tmp.y, sqrt(SQR(tmp.y - x0.y) + SQR(tmp.x - x0.x)));
#   endif   // GRADIENT
#endif // STATS_MAP

//    for(i=0; i<100; i++)
//        printf("f(%f,0.5)=%f\n", 1+(float)i/100., critere(1+(float)i/100., 0.5, &p));

    return 0;
}

