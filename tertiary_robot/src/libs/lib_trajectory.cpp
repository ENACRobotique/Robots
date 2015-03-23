/*
 * lib_trajectory.cpp
 *
 *  Created on: 23 mars 2015
 *      Author: Fab
 */
#include "Arduino.h"
#include "lib_trajectory.h"

int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis){

    if (!(*prev_millis)) *prev_millis=millis();
    move(tab[*i].speed,tab[*i].angle);


    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
    }
    if ( tab[*i].angle==0 && tab[*i].duration==0 && tab[*i].speed==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }

    return 0;
}



