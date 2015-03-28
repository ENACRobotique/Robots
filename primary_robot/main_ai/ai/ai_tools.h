/*
 * obj_fct.h
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */

#ifndef OBJ_FCT_H_
#define OBJ_FCT_H_

#include <ai_types.h>
#include <time_tools.h>

//void printServoPos(eServoPos_t *pos);
void printObsActive(void);
void init_ele(void);
void printListObj(void);
int test_tirette(void);
void simuSecondary(void);
void posPrimary(void);
void checkRobot2Obj(void);
int checkAdvOnRobot(void);
void startColor(void);

#endif /* OBJ_FCT_H_ */
