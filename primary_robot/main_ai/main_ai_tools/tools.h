/*
 * tools.h
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_TOOLS_H_
#define MAIN_AI_TOOLS_TOOLS_H_

#include <NodesNetwork.h>
#include "path.h"
#include "statuses.h"
#include "ihm.h"
#include "net.h"
#include "log.h"
#include "servo.h"
#include "Inbox.h"

#define MINVARIANCE_XY (5e-4) // (cm²)
#define MAXVARIANCE_XY (2e3) // (cm²)
#define MINVARIANCE_THETA (2e-6) // (rad²)
#define MAXVARIANCE_THETA (1e2) // (rad²)


#define MAX_SPEED           12.      //in cm/s
#define MAX_SPEED_ROT       (MAX_SPEED/14)  //in rad/s
#define DELAY_MS            100      //in ms

typedef enum {
    E_AI_SLAVE, E_AI_PROG, E_AI_AUTO
} eAIState_t;

enum class eColor_t{
    YELLOW, GREEN, NONE
};

extern int verbose;
extern int mode_switch;
extern Statuses statuses;
extern Path path;
extern Ihm ihm;
extern Net net;
extern Log logs;
extern Servo servo;
extern Inbox inbox;
extern NodesNetwork nodesNet;

#endif /* MAIN_AI_TOOLS_TOOLS_H_ */
