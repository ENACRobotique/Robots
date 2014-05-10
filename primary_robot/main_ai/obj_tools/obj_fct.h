/*
 * obj_fct.h
 *
 *  Created on: 29 mars 2014
 *      Author: seb
 */

#ifndef OBJ_FCT_H_
#define OBJ_FCT_H_


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "obj.h"
#include "obj_types.h"
#include "obj_fire.h"
#include "obj_time_tools.h"


void printServoPos(eServoPos_t *pos);
void printListObj(void);
void printObsActive(void);
void init_ele(void);
void send_robot(sPath_t path);
int get_position( sPt_t *pos);
int test_in_obs(sPt_t *p);
int test_tirette(void);
void simuSecondary(void);
void posPrimary(void);
void updateRatioObj(int numObj, int robot);
void checkRobot2Obj(void);
int checkAdvOnRobot(void);
int checkRobotBlock(void);
void stopRobot(void);
void startColor(void);

#endif /* OBJ_FCT_H_ */
