/*
 * pos_uncertainty.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_POSTOOLS_POS_UNCERTAINTY_H_
#define LIB_POSTOOLS_POS_UNCERTAINTY_H_

#include <messages-statuses.h>

#ifdef POS_UNCERTAINTY_INTERNALS
#define HAS_POS_UNCERTAINTY_INTERNALS
typedef struct{
    // linear position
    float a, b, c;
    float x, y; // (cm)

    // angular position
    float d;
    float theta; // (rad)
} s2DPUncert_internal;

void gstatus2internal(sGenericPosStatus *i, s2DPUncert_internal *o);
void internal2gstatus(s2DPUncert_internal *i, sGenericPosStatus *o);
#endif

typedef struct{
    float xy_probability;
    float theta_probability;
} s2DPAProbability;

#define MINVARIANCE_XY (1e-3) // (cm²)
#define MAXVARIANCE_XY (8e3) // (cm²)
#define MINVARIANCE_THETA (2e-3) // (rad²)
#define MAXVARIANCE_THETA (7) // (rad²)

void pos_uncertainty_update(sGenericPosStatus *prev, sGenericPosStatus *next);
void pos_uncertainty_mix(sGenericPosStatus *i1, sGenericPosStatus *i2, sGenericPosStatus *o);
s2DPAProbability pos_uncertainty_eval(sGenericPosStatus *i, s2DPosAtt *p);

#endif /* LIB_POSTOOLS_POS_UNCERTAINTY_H_ */
