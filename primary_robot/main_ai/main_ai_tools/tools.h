/*
 * tools.h
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_TOOLS_H_
#define MAIN_AI_TOOLS_TOOLS_H_

#include "path.h"
#include "statuses.h"
#include "ihm.h"
#include "net.h"
#include "log.h"


#define INIT_POS_YELLOW_X 45
#define INIT_POS_YELLOW_Y 100
#define INIT_POS_GREEN_X  (300 - INIT_POS_YELLOW_X)
#define INIT_POS_GREEN_Y  INIT_POS_YELLOW_Y
#define INIT_ANGLE_YELLOW 0
#define INIT_ANGLE_GREEN  M_PI

#define INIT_POS_SLAVE_X 150
#define INIT_POS_SLAVE_Y 100
#define INIT_ANGLE_SLAVE -M_PI_2

#define MAX_SPEED_ROT   M_PI_2  //in rad/s
#define MAX_SPEED       20      //in cm/s
#define DELAY_MS           10      //in ms

typedef enum {
    E_AI_SLAVE, E_AI_PROG, E_AI_AUTO
} eAIState_t;

typedef enum {
    YELLOW, GREEN, NONE
} eColor_t;

extern int verbose;
extern int mode_switch;
extern Statuses statuses;
extern Path path;
extern Ihm ihm;
extern Net net;
extern Log logs;

#endif /* MAIN_AI_TOOLS_TOOLS_H_ */
