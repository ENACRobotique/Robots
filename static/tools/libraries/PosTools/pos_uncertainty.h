/*
 * pos_uncertainty.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_POSTOOLS_POS_UNCERTAINTY_H_
#define LIB_POSTOOLS_POS_UNCERTAINTY_H_

#include <messages-statuses.h>

/**
 * Inverse of covariance matrix to generic position status
 */

typedef struct{
    // linear position
    float a, b, c;
    float x, y; // (cm)

    // angular position
    float d;
    float theta; // (rad)
} s2DPUncert_icovar;

void gstatus2icovar(const sGenericPosStatus *i, s2DPUncert_icovar *o);
void icovar2gstatus(const s2DPUncert_icovar *i, sGenericPosStatus *o);

/**
 * Performing a mix of uncertainty at the inverse covariance matrix level
 */

void icovar_mix(const s2DPUncert_icovar *i1, const s2DPUncert_icovar *i2, s2DPUncert_icovar *o);

/**
 * Covariance matrix to generic position status
 */

typedef struct{
    // linear position
    float a, b, c; // (cm²)
    float x, y; // (cm)

    // angular position
    float d; // (rad²)
    float theta; // (rad)
} s2DPUncert_covar;

void covar2gstatus(const s2DPUncert_covar *i, sGenericPosStatus *o);
void gstatus2covar(const sGenericPosStatus *i, s2DPUncert_covar *o);

/*
 * Performing the matrix inversion
 */

void covar2icovar(const s2DPUncert_covar *i, s2DPUncert_icovar *o);
void icovar2covar(const s2DPUncert_icovar *i, s2DPUncert_covar *o);

/**
 * Evaluating the probability of a position of the robot
 */

typedef struct{
    float xy_probability;
    float theta_probability;
} s2DPAProbability;

s2DPAProbability pos_uncertainty_eval(const sGenericPosStatus *i, const s2DPosAtt *p);

/**
 * Performing a high level mix of uncertainty
 */

#define MINVARIANCE_XY (5e-4) // (cm²)
#define MAXVARIANCE_XY (2e3) // (cm²)
#define MINVARIANCE_THETA (2e-6) // (rad²)
#define MAXVARIANCE_THETA (1e2) // (rad²)

void pos_uncertainty_mix(const sGenericPosStatus *i1, const sGenericPosStatus *i2, sGenericPosStatus *o);

#endif /* LIB_POSTOOLS_POS_UNCERTAINTY_H_ */
