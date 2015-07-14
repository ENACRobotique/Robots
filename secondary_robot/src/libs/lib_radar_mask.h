/*
 * lib_radar_mask.h
 *
 *  Created on: 2015
 *      Author: Fab
 */

#ifndef LIB_RADAR_MASK_H_
#define LIB_RADAR_MASK_H_

#include "state_types.h"


typedef struct {
    int limit_start;      // in cm
    int limit_end;        //in cm
    unsigned long duration;   //in ms
}radarElem;


extern int _backFromPauseRadar;

int periodicProgRadarLimit(radarElem tab[], unsigned long *pausetime, int *i, unsigned long *prev_millis);

#endif /* LIB_RADAR_MASK_H_ */
