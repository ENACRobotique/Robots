/*
 * main.h
 *
 *  Created on: 29 mai 2014
 *      Author: seb
 */

#ifndef MAIN_H_
#define MAIN_H_

extern sPath_t curr_path;
extern int curr_traj_extract_sid;
extern int last_tid;

typedef enum{
    E_AI_SLAVE,
    E_AI_PROG,
    E_AI_AUTO,
    E_AI_FIRE
} eAIState_t;

#endif /* MAIN_H_ */
