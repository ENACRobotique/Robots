/*
 * variables.h
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_VARIABLES_H_
#define MAIN_AI_TOOLS_VARIABLES_H_

#include <main_ai_tools/path.h>
#include <main_ai_tools/statuses.h>


#define INIT_POS_YELLOW_X 45
#define INIT_POS_YELLOW_Y 100
#define INIT_POS_GREEN_X  (300 - INIT_POS_YELLOW_X)
#define INIT_POS_GREEN_Y  INIT_POS_YELLOW_Y
#define INIT_ANGLE_YELLOW 0
#define INIT_ANGLE_GREEN  M_PI

#define INIT_POS_SLAVE_X 150
#define INIT_POS_SLAVE_Y 100
#define INIT_ANGLE_SLAVE -M_PI_2

typedef enum {
    E_AI_SLAVE, E_AI_PROG, E_AI_AUTO
} eAIState_t;

extern Statuses statuses;
extern Path path;


#endif /* MAIN_AI_TOOLS_VARIABLES_H_ */
