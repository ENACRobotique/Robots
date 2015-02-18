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

void send_robot(sPath_t path);
//void printServoPos(eServoPos_t *pos);
void printListObj(void);
void printObsActive(void);
void init_ele(void);
int get_position(sPt_t *pos);
int test_in_obs(sPt_t *p);
int test_tirette(void);
void simuSecondary(void);
void posPrimary(void);
void updateRatioObj(int numObj, int robot);
void checkRobot2Obj(void);
int checkAdvOnRobot(void);
int checkRobotBlock(void);
void startColor(void);
int initTraj(void);
int newSpeed(float speed);
int setPos(sPt_t *p, sNum_t theta);

#endif /* OBJ_FCT_H_ */
