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


typedef struct {
    int speed;      // speed/1.77 = real_speed in cm/s
    int teta;      //in ° (degrees)
    unsigned int duration;   //in ms <60s
}trajElem;

//pointeur de fonction
typedef int(*periodicTraj)(const trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);

extern int _backFromPause;

int periodicProgTrajHeading(const trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);

int periodicProgTraj(const trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);
#endif /* STATE_TRAJECTORY_H_ */
