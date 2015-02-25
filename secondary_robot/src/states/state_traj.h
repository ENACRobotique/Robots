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

#define MUR_R
//#define Int_R
#define MUR_Y
//#define INT_Y

typedef struct {
    int speed;      // speed/1.77 = real_speed in cm/s
    int omega;      //in ... ?
    unsigned long duration;   //in ms
}trajElem;

int progTraj(trajElem tab[]);


extern sState sTrajRedInit;
extern sState sTrajYellowInit;
extern sState sTrajEndStairsYellow;
extern sState sTrajEndStairsGreen;


int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);



#endif /* STATE_TRAJ_H_ */
