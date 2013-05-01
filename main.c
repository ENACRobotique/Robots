#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

#define SQR(v) ((v)*(v))
#define MIN(a, b) ((a)>(b)?(b):(a))
#define MAX(a, b) ((a)>(b)?(a):(b))

#define GAIN_ADAPT
//#define FIXED_SEED 1
//#define FIXED_INPUT_PROBLEM
#define WARN_JUMP
#define NELDERMEAD

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

void simu_perception(float x, float y, sPerception *p) {
    if(!p)
        return;

    // distances
    p->d1 = sqrt(SQR(x - glob_params.B1.x) + SQR(y - glob_params.B1.y));
    p->d2 = sqrt(SQR(x - glob_params.B2.x) + SQR(y - glob_params.B2.y));
    p->d3 = sqrt(SQR(x - glob_params.B3.x) + SQR(y - glob_params.B3.y));

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

float critere(float x, float y, sPerception *p) {
    sPerception simu_p;

    simu_perception(x, y, &simu_p);

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
sPt_t neldermead(sPt_t center, float range, sPerception *p) {
    sPt_t xb, xr, xe, xc, x[3];
    int i, j, i_max, i_MAX, i_MIN;
    float z[3], z_xr, z_xe, z_xc, alpha=1., gamma=2., rho=-0.5, sigma=0.5;

    #define Z_CRIT (18)

    for(j=0; j<3; j++) {
        x[j].x = center.x + range*cos(120.*(float)j*M_PI/180.);
        x[j].y = center.y + range*sin(120.*(float)j*M_PI/180.);
        z[j] = critere(x[j].x, x[j].y, p);

        if(z[j] < Z_CRIT)
            return x[j];
    }

    for(i=0; i<1000; i++) {
        for(j=0; j<3; j++)
            printf("x[%u]=(%.2f, %.2f, %.2f)\n", j, x[j].x, x[j].y, z[j]);

// tri
        i_max = 0;
        i_MAX = 0;
        i_MIN = 0;
        for(j=1; j<3; j++) {
            if(z[j] < z[i_MIN])
                i_MIN = j;
            if(z[j] > z[i_MAX]) {
                i_max = i_MAX;
                i_MAX = j;
            }
            else if(z[j] > z[i_max])
                i_max = j;
        }
        printf("i_MIN = %u\ni_max = %u\ni_MAX = %u\n", i_MIN, i_max, i_MAX);

// barycentre
        xb.x = 0;
        xb.y = 0;
        for(j=0; j<3; j++) {
            if(j != i_MAX) {
                xb.x += x[j].x/2.;
                xb.y += x[j].y/2.;
            }
        }

        printf("xb=(%.2f, %.2f)\n", xb.x, xb.y);

// calcul point réflechi
        xr.x = xb.x + alpha*(xb.x - x[i_MAX].x);
        xr.y = xb.y + alpha*(xb.y - x[i_MAX].y);
        z_xr = critere(xr.x, xr.y, p);

        printf("xr=(%.2f, %.2f, %.2f)\n", xr.x, xr.y, z_xr);
        if(z_xr < Z_CRIT)
            return xr;

// actualisation simplexe
        // reflexion
        if(z[i_MIN] <= z_xr && z_xr < z[i_max]) {
            x[i_MAX] = xr;
            z[i_MAX] = z_xr;
            continue;
        }

        // expansion
        if(z_xr < z[i_MIN]) {
            xe.x = xb.x + gamma*(xb.x - x[i_MAX].x);
            xe.y = xb.y + gamma*(xb.y - x[i_MAX].y);
            z_xe = critere(xe.x, xe.y, p);

            printf("xe=(%.2f, %.2f, %.2f)\n", xe.x, xe.y, z_xe);
            if(z_xe < Z_CRIT)
                return xe;

            if(z_xe < z_xr) {
                x[i_MAX] = xe;
                z[i_MAX] = z_xe;
            }
            else {
                x[i_MAX] = xr;
                z[i_MAX] = z_xr;
            }
            continue;
        }

        // contraction
        xc.x = xb.x + rho*(xb.x - x[i_MAX].x);
        xc.y = xb.y + rho*(xb.y - x[i_MAX].y);
        z_xc = critere(xc.x, xc.y, p);

        printf("xc=(%.2f, %.2f, %.2f)\n", xc.x, xc.y, z_xc);
        if(z_xc < Z_CRIT)
            return xc;

        if(z_xc < z_xr) {
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
            z[j] = critere(x[j].x, x[j].y, p);
            if(z[j] < Z_CRIT)
                return x[j];
        }
    }

    x[0].x = x[0].y = 0;
    return x[0];
}
#endif

int main() {
    float x0, y0, x, y, z, step, alpha, tmp;
#if defined(GAIN_ADAPT)
    float z_old, x_old, y_old;
#endif
#if defined(WARN_JUMP)
    float z_prev;
#endif
    sVec_t G;
    sPerception p;
    int i;
#ifdef NELDERMEAD
    sPt_t res, c;
#endif

    // entry point
    init_globals();

    // builds the noisy perception data
#ifdef FIXED_INPUT_PROBLEM
    x0 = 1.5;
    y0 = 0.5;
#else
    x0 = 2.8;
    y0 = 0.8;
#endif
    simu_perception(x0, y0, &p);
    estim_incertitude(&p);
#ifdef FIXED_INPUT_PROBLEM
    p.d1 = 1.60459;
    p.d2 = 2.27835;
    p.d3 = 1.60727;
    p.a12 = 1.1214;
    p.a23 = 2.04323;
    p.a31 = 3.11656;
#else
    bruite_perception(&p);
#endif

    // add some bias to the start point
#ifdef FIXED_INPUT_PROBLEM
    x = 1.519;
    y = 0.48867;
#else
    tmp = uni_rand(-M_PI, M_PI);
    x = x0 + 0.02*cos(tmp);
    y = y0 + 0.02*sin(tmp);
    printf("tmp=%.1f°\n", tmp*180./M_PI);
#endif

#ifdef NELDERMEAD
    c.x = x;
    c.y = y;
    res = neldermead(c, 0.05, &p);

    printf("res=(%.2f, %.2f) (erreur de %.6fm)\n", res.x, res.y, sqrt(SQR(res.y-y0) + SQR(res.x-x0)));

    return 0;
#endif

    // gradient descent
    step = 1e-3;
    alpha = 1e-7;
    printf("u_d1=%.3fm, u_d2=%.3fm, u_d3=%.3fm\n", p.u_d1, p.u_d2, p.u_d3);
    printf("u_a12=%.2f°, u_a23=%.2f°, u_a31=%.2f°\n", p.u_a12*180./M_PI, p.u_a23*180./M_PI, p.u_a31*180./M_PI);

    for(i=0; i<1000; i++) {
        z = critere(x, y, &p);

#ifdef GAIN_ADAPT
        if(!i || z <= z_old) {
printf("###ok    (%f <= %f)### ", z, z_old);
            alpha *= 1.2;
            x_old = x;
            y_old = y;
            z_old = z;
        }
        else {
printf("###failed (%f > %f)### ", z, z_old);
            alpha *= 0.5;
            x = x_old;
            y = y_old;
            z = z_old;
        }
#endif

        G.x = (critere(x + step, y, &p) - z)/step;
        G.y = (critere(x, y + step, &p) - z)/step;
        tmp = sqrt(SQR(G.x) + SQR(G.y));

        printf("IT%03u: en %.3fm, %.3fm: %03.0f, %f, dir %.2f°", i, x, y, z, tmp, 180.*atan2(-G.y, -G.x)/M_PI);
#ifdef GAIN_ADAPT
        printf(", alpha %e", alpha);
#endif
#ifdef WARN_JUMP
        if(i && z > z_prev)
            printf(", /!\\ bad step /!\\");
        z_prev = z;
#endif
        printf("\n");

        if(z < 15)//tmp < 2e3)
            break;

        x -= alpha*G.x;
        y -= alpha*G.y;
    }

    printf("%u itérations\n", i);
    printf("p(%.6fm, %.6fm) (erreur de %.6fm)\n", x, y, sqrt(SQR(y-y0) + SQR(x-x0)));

//    for(i=0; i<100; i++)
//        printf("f(%f,0.5)=%f\n", 1+(float)i/100., critere(1+(float)i/100., 0.5, &p));

    return 0;
}

