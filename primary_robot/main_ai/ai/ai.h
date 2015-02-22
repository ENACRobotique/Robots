/*
 * ai.hpp
 *
 *  Created on: 11 févr. 2015
 *      Author: seb
 */

#ifndef AI_AI_H_
#define AI_AI_H_

#include <ai_types.h>
#include <main_ai_tools/statuses.h>
#include <tools.h>
#include "obj.h"
#include <vector>
#include <cmath>
#define MAX_RETRIES 1

//#define ABS_POS


void obj_step(eAIState_t AIState);
extern int obj_init(eAIState_t AIState);

extern std::vector<Obj*> listObj;
extern Path path_;
extern Statuses statuses_;

#endif /* AI_AI_H_ */
