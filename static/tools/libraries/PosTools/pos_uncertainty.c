/*
 * pos_uncertainty.c
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#include <assert.h>
#include <math.h>
#include <messages-elements.h>
#include <messages-position.h>

#include "pos_uncertainty.h"

#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)>(b)?(b):(a))
#define CLAMP(m, v, M) MAX((m), MIN((v), (M)))

// see static/locomotion/simu for design files

void gstatus2icovar(const sGenericPosStatus *i, s2DPUncert_icovar *o){
    // linear position
    // Converts rotated 2D gaussian to quadratic form coefficients
    //   input:  f(x, y) = X²/var_x + Y²/var_y
    //               where: X =  cos(an)*x + sin(an)*y
    //                      Y = -sin(an)*x + cos(an)*y
    //               where: cos(an) = ca
    //                      sin(an) = sa
    //   output: f(x, y) = ax² + 2bxy + cy² (formalized as inverse covariance: X^T * M^-1 * X)
    float ca = cosf(i->pos_u.a_angle), sa = sinf(i->pos_u.a_angle);
    float a_var = CLAMP(MINVARIANCE_XY, i->pos_u.a_var, MAXVARIANCE_XY);
    float b_var = CLAMP(MINVARIANCE_XY, i->pos_u.b_var, MAXVARIANCE_XY);
    o->a = ca*ca/a_var + sa*sa/b_var;
    o->b = ca*sa*(1/a_var - 1/b_var);
    o->c = sa*sa/a_var + ca*ca/b_var;
    o->x = i->pos.x;
    o->y = i->pos.y;

    // angular position
    o->d = 1/CLAMP(MINVARIANCE_THETA, i->pos_u.theta_var, MAXVARIANCE_THETA);
    o->theta = i->pos.theta;
}

void icovar2gstatus(const s2DPUncert_icovar *i, sGenericPosStatus *o){
    // linear position
    // Converts quadratic form coefficients to rotated 2D gaussian
    //   input:  f(x, y) = ax² + 2bxy + cy² (formalized as inverse covariance: X^T * M^-1 * X)
    //   output: f(x, y) = X²/var_x + Y²/var_y
    //               where: X =  cos(an)*x + sin(an)*y
    //                      Y = -sin(an)*x + cos(an)*y
    float sqrt_delta = sqrtf((i->a - i->c)*(i->a - i->c) + 4*i->b*i->b);

    int noangle = 0;
    if(i->b == 0.f || fabsf(i->b) > 1e5*sqrt_delta){
        noangle = 1;
    }

    if(i->c < i->a){
        sqrt_delta = - sqrt_delta;
    }
    float trace = i->a + i->c;
    float two_det = 2.f*(i->a*i->c - i->b*i->b);
    o->pos_u.a_var = (trace + sqrt_delta)/two_det;
    o->pos_u.b_var = (trace - sqrt_delta)/two_det;
    o->pos_u.a_angle = noangle ? 0.f : -asinf(CLAMP(-1.f, 2*i->b/sqrt_delta, 1.f))/2;
    o->pos.x = i->x;
    o->pos.y = i->y;

    // angular position
    o->pos_u.theta_var = 1/i->d;
    o->pos.theta = i->theta;
}

void icovar_mix(const s2DPUncert_icovar *i1, const s2DPUncert_icovar *i2, s2DPUncert_icovar *o){
    // actual calculation (linear position)
    o->a = i1->a + i2->a;
    o->b = i1->b + i2->b;
    o->c = i1->c + i2->c;

    float den = o->a*o->c - o->b*o->b;
    float G = -(i2->a*(i2->x - i1->x) + i2->b*(i2->y - i1->y));
    float H = -(i2->b*(i2->x - i1->x) + i2->c*(i2->y - i1->y));
    o->x = (H*o->b - G*o->c) / den + i1->x;
    o->y = (G*o->b - H*o->a) / den + i1->y;

    // actual calculation (angular position)
    o->d = i1->d + i2->d;

    float mn_theta = i2->theta;
    while(i1->theta - mn_theta > M_PI) mn_theta += 2*M_PI;
    while(i1->theta - mn_theta < -M_PI) mn_theta -= 2*M_PI;
    o->theta = (i1->d*i1->theta + i2->d*mn_theta)/o->d;
}

void covar2gstatus(const s2DPUncert_covar *i, sGenericPosStatus *o){
    // linear position
    float trace = i->a + i->c;
#if 0
    float det = i->a*i->c - i->b*i->b;
    float sqrt_delta = sqrtf(trace*trace - 4*det);
#else
    float sqrt_delta = sqrtf((i->a - i->c)*(i->a - i->c) + 4*i->b*i->b);
#endif

    int noangle = 0;
    if(i->b == 0.f || fabsf(i->b) > 1e5*sqrt_delta){
        noangle = 1;
    }

    if(i->c > i->a){
        sqrt_delta = -sqrt_delta;
    }
    o->pos_u.a_var = (trace + sqrt_delta)/2;
    o->pos_u.b_var = trace - o->pos_u.a_var;
    o->pos_u.a_angle = noangle ? 0.f : asinf(CLAMP(-1.f, 2*i->b/sqrt_delta, 1.f))/2;
    o->pos.x = i->x;
    o->pos.y = i->y;

    // angular position
    o->pos_u.theta_var = i->d;
    o->pos.theta = i->theta;
}

void gstatus2covar(const sGenericPosStatus *i, s2DPUncert_covar *o){
    // linear position
    float caa = cosf(i->pos_u.a_angle);
    float saa = sinf(i->pos_u.a_angle);

    o->a = caa*caa*i->pos_u.a_var + saa*saa*i->pos_u.b_var;
    o->b = caa*saa*(i->pos_u.a_var - i->pos_u.b_var);
    o->c = caa*caa*i->pos_u.b_var + saa*saa*i->pos_u.a_var;
    o->x = i->pos.x;
    o->y = i->pos.y;

    // angular position
    o->d = i->pos_u.theta_var;
    o->theta = i->pos.theta;
}

void covar2icovar(const s2DPUncert_covar *i, s2DPUncert_icovar *o){
    // linear position
    float det = i->a*i->c - i->b*i->b;
    o->a = i->c/det;
    o->b = -i->b/det;
    o->c = i->a/det;
    o->x = i->x;
    o->y = i->y;

    // angular position
    o->d = 1/i->d;
    o->theta = i->theta;
}

void icovar2covar(const s2DPUncert_icovar *i, s2DPUncert_covar *o){
    // linear position
    float det = i->a*i->c - i->b*i->b;
    o->a = i->c/det;
    o->b = -i->b/det;
    o->c = i->a/det;
    o->x = i->x;
    o->y = i->y;

    // angular position
    o->d = 1/i->d;
    o->theta = i->theta;
}

s2DPAProbability pos_uncertainty_eval(const sGenericPosStatus *i, const s2DPosAtt *p){
    assert(i->pos.frame == p->frame);

    s2DPUncert_icovar ii;
    gstatus2icovar(i, &ii);

    s2DPAProbability o;
    float dx = p->x - ii.x;
    float dy = p->y - ii.y;
    float dtheta = p->theta - ii.theta;

    while(dtheta > M_PI) dtheta -= 2*M_PI;
    while(dtheta < -M_PI) dtheta += 2*M_PI;

    o.xy_probability = expf(-(ii.a*dx*dx + 2*ii.b*dx*dy + ii.c*dy*dy)/2.f)/(2*M_PI*sqrtf(i->pos_u.a_var*i->pos_u.b_var));
    o.theta_probability = expf(-ii.d*dtheta*dtheta/2.f)/sqrtf(2*M_PI*i->pos_u.theta_var);

    return o;
}

void pos_uncertainty_mix(const sGenericPosStatus *i1, const sGenericPosStatus *i2, sGenericPosStatus *o){
    s2DPUncert_icovar pg, mn, nw;

    // necessary verifications
    assert(i1 && i2 && o);
    assert(i1->pos.frame == i2->pos.frame);
    assert(i1->id == i2->id);
    assert(i1->date == i2->date);

    // get input data
    gstatus2icovar(i1, &pg);
    gstatus2icovar(i2, &mn);

    // actual computation
    icovar_mix(&pg, &mn, &nw);

    // fill output
    o->id = i1->id;
    o->date = i1->date;
    icovar2gstatus(&nw, o);
    o->pos.frame = i1->pos.frame;
}
