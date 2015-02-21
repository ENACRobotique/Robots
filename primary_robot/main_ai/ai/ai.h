/*
 * ai.hpp
 *
 *  Created on: 11 févr. 2015
 *      Author: seb
 */

#ifndef AI_AI_H_
#define AI_AI_H_

#include <ai_types.h>
#include "obj.h"

#include <vector>
#include <cmath>

#define INIT_POS_YELLOW_X 45
#define INIT_POS_YELLOW_Y 100
#define INIT_POS_GREEN_X  (300 - INIT_POS_YELLOW_X)
#define INIT_POS_GREEN_Y  INIT_POS_YELLOW_Y
#define INIT_ANGLE_YELLOW 0
#define INIT_ANGLE_GREEN  M_PI

#define MAX_RETRIES 1

//#define ABS_POS
typedef enum {
    E_AI_SLAVE, E_AI_PROG, E_AI_AUTO
} eAIState_t;


void obj_step(eAIState_t AIState);
extern int obj_init(eAIState_t AIState);

extern std::vector<Obj*> listObj;

#endif /* AI_AI_H_ */
