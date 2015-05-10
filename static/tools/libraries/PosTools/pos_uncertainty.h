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
    float a, b, c;
    float x, y; // (cm)
} s2DPUncert_internal;

void gstatus2internal(sGenericPosStatus *i, s2DPUncert_internal *o);
void internal2gstatus(s2DPUncert_internal *i, sGenericPosStatus *o);
#endif

void pos_uncertainty_update(sGenericPosStatus *prev, sGenericPosStatus *next);
void pos_uncertainty_mix(sGenericPosStatus *i1, sGenericPosStatus *i2, sGenericPosStatus *o);

#endif /* LIB_POSTOOLS_POS_UNCERTAINTY_H_ */
