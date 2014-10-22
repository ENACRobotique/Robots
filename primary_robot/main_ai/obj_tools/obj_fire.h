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

//Bonus Fire
#define AZ 22
#define BZ 32
#define BC 10
#define BD 25
#define EZ 10

extern eServoPos_t armLeft, armRight;

void obj_fire(iABObs_t  obj);
void createEPfire(sPt_t *pt, sNum_t theta, sNum_t r, int numObj);
void createEPfire2(int numObj);
int interC2D(const sLin_t *l, const sPt_t *c, sNum_t r, sPt_t *pt1, sPt_t *pt2);
int objBonusFire(void);
void revertFireDemo(void);


#endif /* OBJ_FIRE_H_ */
