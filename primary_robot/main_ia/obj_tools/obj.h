/*
 * obj.h
 *
 *  Created on: 17 mars 2014
 *      Author: ludo6431
 */

#ifndef OBJ_H_
#define OBJ_H_

#include "math_types.h"
#include "millis.h"

void obj_step();
int obj_init();

extern sNum_t theta_robot;
extern sNum_t speed;
extern int mode_obj;
extern sPt_t pt_select;
#endif /* OBJ_H_ */
