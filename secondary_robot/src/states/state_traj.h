/*
 * state_traj.h
 *
 *  Created on: 8 mai 2013
 *      Author: quentin
 *   Modify on: janvier 2014
 *   		By: Seb
 */

#ifndef STATE_TRAJ_H_
#define STATE_TRAJ_H_

#include "state_types.h"
#include "lib_trajectory.h"


//pointeur de fonction
typedef int(*periodicTraj)(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);

int progTraj(trajElem tab[]);


extern sState sTrajGreenInit;
extern sState sTrajPurpleInit;
extern sState sTrajEndStairsGreen;
extern sState sTrajEndStairsYellow;

extern sState sTrajGreenInit;
extern sState sTrajPurpleInit;



#endif /* STATE_TRAJ_H_ */
