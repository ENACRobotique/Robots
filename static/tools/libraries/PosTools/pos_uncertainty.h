/*
 * pos_uncertainty.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_POSTOOLS_POS_UNCERTAINTY_H_
#define LIB_POSTOOLS_POS_UNCERTAINTY_H_

#include <messages-statuses.h>

void pos_uncertainty_update(sGenericStatus *prev, sGenericStatus *next);
void pos_uncertainty_mix(sGenericStatus *i1, sGenericStatus *i2, sGenericStatus *o);

#endif /* LIB_POSTOOLS_POS_UNCERTAINTY_H_ */
