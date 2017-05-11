/*
 * state_traj.h
 *
 *  Created on: 8 mai 2013
 *      Author: quentin
 *   Modify on: janvier 2014
 *   		By: Seb
 */

#ifndef LIB_TRAJECTORY_H_
#define LIB_TRAJECTORY_H_

#include "state_types.h"

typedef  enum {
	DISTANCE,
	TEMPS,
}traj_mode;

typedef struct {
    int speed;      // speed/1.77 = real_speed in cm/s
    int teta;      //in ° (degrees)
    double value;   //in ms <60s
    traj_mode mode;
}trajElem;

#define TRAJ_CM2ACCU 1036.269
//quart de tour trigo
#define QUART_TOUR_POS {0,90,250,TEMPS},{250,90,+71.5/4,DISTANCE},{0,0,250,TEMPS}
#define QUART_TOUR_NEG {0,90,250,TEMPS},{250,90,-71.5/4,DISTANCE},{0,0,250,TEMPS}

//pointeur de fonction
typedef int(*periodicTraj)(const trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);

extern int _backFromPause;

int periodicProgTrajHeading(const trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);

int periodicProgTraj(const trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);
#endif /* STATE_TRAJECTORY_H_ */
