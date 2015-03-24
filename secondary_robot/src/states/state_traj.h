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

#define MUR_R
//#define Int_R
#define MUR_Y
//#define INT_Y

int progTraj(trajElem tab[]);


extern sState sTrajGreenInit;
extern sState sTrajYellowInit;
extern sState sTrajEndStairsGreen;
extern sState sTrajEndStairsYellow;

extern sState sTrajGreenInit;
extern sState sTrajYellowInit;


#endif /* STATE_TRAJ_H_ */
