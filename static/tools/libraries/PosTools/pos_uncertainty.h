/*
 * pos_uncertainty.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_POSTOOLS_POS_UNCERTAINTY_H_
#define LIB_POSTOOLS_POS_UNCERTAINTY_H_

#include <messages-statuses.h>

typedef struct{
    // linear position
    float a, b, c;
    float x, y; // (cm)

    // angular position
    float d;
    float theta; // (rad)
} s2DPUncert_icovar;

void gstatus2icovar(sGenericPosStatus *i, s2DPUncert_icovar *o);
void icovar2gstatus(s2DPUncert_icovar *i, sGenericPosStatus *o);

typedef struct{
    // linear position
    float a, b, c;
    float x, y; // (cm)

    // angular position
    float d;
    float theta; // (rad)
} s2DPUncert_covar;

void covar2gstatus(s2DPUncert_covar *i, sGenericPosStatus *o);
void gstatus2covar(sGenericPosStatus *i, s2DPUncert_covar *o);

typedef struct{
    float xy_probability;
    float theta_probability;
} s2DPAProbability;

#define MINVARIANCE_XY (5e-4) // (cm²)
#define MAXVARIANCE_XY (2e3) // (cm²)
#define MINVARIANCE_THETA (2e-3) // (rad²)
#define MAXVARIANCE_THETA (500) // (rad²)

void pos_uncertainty_mix(sGenericPosStatus *i1, sGenericPosStatus *i2, sGenericPosStatus *o);
s2DPAProbability pos_uncertainty_eval(sGenericPosStatus *i, s2DPosAtt *p);

#endif /* LIB_POSTOOLS_POS_UNCERTAINTY_H_ */
