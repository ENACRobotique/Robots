/*
 * obj_fire.h
 *
 *  Created on: 17 avr. 2014
 *      Author: seb
 */

#ifndef OBJ_FIRE_H_
#define OBJ_FIRE_H_

#include <math.h>
#include <string.h>

#include "tools.h"
#include "millis.h"
#include "obj_types.h"
#include "obj_geometry_tools.h"
#include "obj_fct.h"

void obj_fire(iABObs_t  obj);
void createEPfire(sPt_t *pt, sNum_t theta, sNum_t r, int numObj);
void createEPfire2(int numObj);


#endif /* OBJ_FIRE_H_ */
