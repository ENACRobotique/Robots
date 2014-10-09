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

void gstatus2internal(sGenericStatus *i, s2DPUncert_internal *o);
void internal2gstatus(s2DPUncert_internal *i, sGenericStatus *o);
#endif

void pos_uncertainty_update(sGenericStatus *prev, sGenericStatus *next);
void pos_uncertainty_mix(sGenericStatus *i1, sGenericStatus *i2, sGenericStatus *o);

#endif /* LIB_POSTOOLS_POS_UNCERTAINTY_H_ */
