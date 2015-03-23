/*
 * lib_trajectory.h
 *
 *  Created on: 23 mars 2015
 *      Author: Fab
 */

#ifndef LIB_TRAJECTORY_H_
#define LIB_TRAJECTORY_H_

#include "lib_move.h"
#include "Arduino.h"

typedef struct {
    int speed;      //in inc/s
    int angle;      //in degree, 0°=straight forward
    unsigned long duration;   //in ms
}trajElem;

int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);

#endif /* LIB_TRAJECTORY_H_ */
