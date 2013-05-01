#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

#define SQR(v) ((v)*(v))
#define MIN(a, b) ((a)>(b)?(b):(a))
#define MAX(a, b) ((a)>(b)?(a):(b))

//#define FIXED_SEED 1
//#define FIXED_INPUT_PROBLEM
#define Z_CRIT 18

#define NELDERMEAD
#ifdef NELDERMEAD
#   define NM_DEBUG
#endif

//#define GRADIENT
#ifdef GRADIENT
#   define GR_GAIN_ADAPT

#   define GR_DEBUG
#   ifdef GR_DEBUG
#       define GR_WARN_JUMP
#   endif
#endif

typedef struct {
// distances aux balises (m)
    float d1;
    float d2;
    float d3;

    float u_d1;
    float u_d2;
    float u_d3;

// angles inter-balises (rad)
    float a12;
    float a23;
    float a31;

    float u_a12;
    float u_a23;
    float u_a31;
} sPerception;

typedef struct {
    float x;
    float y;
} sPt_t;
typedef sPt_t sVec_t;

struct {
    // position balises (à calibrer à chaque début de match)
    sPt_t B1, B2, B3;

    float D12;  // distance B1-B2
    float D23;  // distance B2-B3
    float D31;  // distance B3-B1

    float omega;    // vitesse de rotation du rotor (rad/s)
    float r; // rayon du cercle sur lequel sont les deux lasers (m)
    float epsilon;  // erreur de parallélisme des lasers (rad)

    // paramètres d'incertitude
    float u_delta_t_distance;  // (s)
    float u_delta_t_angle;  // (s)
    float u_omega;  // (rad/s)
} glob_params;

void init_globals() {
    glob_params.B1.x = -0.04;
    glob_params.B1.y = -0.04;
    glob_params.B2.x = -0.04;
    glob_params.B2.y =  2.04;
    glob_params.B3.x =  3.04;
    glob_params.B3.y =  1.;

    glob_params.D12 = sqrt(SQR(glob_params.B2.x - glob_params.B1.x) + SQR(glob_params.B2.y - glob_params.B1.y));
    glob_params.D23 = sqrt(SQR(glob_params.B3.x - glob_params.B2.x) + SQR(glob_params.B3.y - glob_params.B2.y));
    glob_params.D31 = sqrt(SQR(glob_params.B1.x - glob_params.B3.x) + SQR(glob_params.B1.y - glob_params.B3.y));

    glob_params.omega = 20.*2.*M_PI;
    glob_params.r = 25e-3;
    glob_params.epsilon = 0.5*M_PI/180.;
    
    glob_params.u_delta_t_distance = 8e-6;
    glob_params.u_delta_t_angle = 16e-6;
    glob_params.u_omega = SQR(glob_params.omega)/(2.*M_PI)*4e-6;

#ifdef FIXED_SEED
    srand(FIXED_SEED);
#else
    srand(time(NULL) + (int)getpid());
#endif
}

float uni_rand(float min, float max) {
    return min + ((max-min)*(float)rand())/(float)RAND_MAX;
}

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
    p->u_d1 = abs(-p->d1*glob_params.omega*sqrt(SQR(p->d1) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_delta_t_distance + abs(-p->d1*delta_t*sqrt(SQR(p->d1) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_omega;

    alpha = 2*asin(glob_params.r/p->d2) + glob_params.epsilon;    // angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / glob_params.omega;    // temps correspondant à l'angle alpha ci-dessus (s)
    p->u_d2 = abs(-p->d2*glob_params.omega*sqrt(SQR(p->d2) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_delta_t_distance + abs(-p->d2*delta_t*sqrt(SQR(p->d2) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_omega;

    alpha = 2*asin(glob_params.r/p->d3) + glob_params.epsilon;    // angle de rotation du rotor nécessaire pour détecter, au niveau du récepteur, le second laser en partant du premier (rad)
    delta_t = alpha / glob_params.omega;    // temps correspondant à l'angle alpha ci-dessus (s)
    p->u_d3 = abs(-p->d3*glob_params.omega*sqrt(SQR(p->d3) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_delta_t_distance + abs(-p->d3*delta_t*sqrt(SQR(p->d3) - SQR(glob_params.r))/(2 * glob_params.r))*glob_params.u_omega;

    // incertitudes sur angles
    delta_t = p->a12/glob_params.omega;
    p->u_a12 = abs(glob_params.omega)*glob_params.u_delta_t_angle + abs(delta_t)*glob_params.u_omega;

    delta_t = p->a23/glob_params.omega;
    p->u_a23 = abs(glob_params.omega)*glob_params.u_delta_t_angle + abs(delta_t)*glob_params.u_omega;

    delta_t = p->a31/glob_params.omega;
    p->u_a31 = abs(glob_params.omega)*glob_params.u_delta_t_angle + abs(delta_t)*glob_params.u_omega;
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

#ifdef NELDERMEAD
void neldermead(sPt_t *x0, float range, sPerception *p) {
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
            return;
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
            return;
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
                return;
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
            return;
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
                return;
            }
        }
    }

// too much iterations
    x0->x = x0->y = -1;
}
#endif

#ifdef GRADIENT
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
#endif

int main(int argc, char *argv[]) {
    sPt_t x0 /* reference */, x /* start point */, tmp;
    sPerception p;
#if !defined(FIXED_INPUT_PROBLEM)
    float dir;
#endif
//    int i;

    // entry point
    init_globals();

    // builds the noisy perception data
#ifdef FIXED_INPUT_PROBLEM
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
#else
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
#endif

#ifdef NELDERMEAD
    tmp = x;
    nb_cri = 0;

    neldermead(&tmp, 0.05, &p);

    printf("%u calculs du critère\n", nb_cri);
    printf("nm_res=(%.2f, %.2f) (erreur de %.6fm)\n", tmp.x, tmp.y, sqrt(SQR(tmp.y - x0.y) + SQR(tmp.x - x0.x)));
#endif

#ifdef GRADIENT
    tmp = x;
    nb_cri = 0;

    gradient(&tmp, &p);

    printf("%u calculs du critère\n", nb_cri);
    printf("gr_res=(%.2f, %.2f) (erreur de %.6fm)\n", tmp.x, tmp.y, sqrt(SQR(tmp.y - x0.y) + SQR(tmp.x - x0.x)));
#endif

//    for(i=0; i<100; i++)
//        printf("f(%f,0.5)=%f\n", 1+(float)i/100., critere(1+(float)i/100., 0.5, &p));

    return 0;
}

