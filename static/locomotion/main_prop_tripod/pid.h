/*
 * pid.h
 *
 *  Created on: 11 févr. 2015
 *      Authors: Ludo, Yoyo
 */

#ifndef _PID_H
#define _PID_H

typedef struct {
    unsigned char shift;

    int KP, KI, KD;

    int maxErr, sum, maxSum, I_max;
    int lastProcessValue;
} PID_t;

// Proportional factor (KP)
// Integral factor (KI=KP*T/Ti)
// Derivative factor (KD=KP*Td/T)

void pid_init  (PID_t *p, int KP, int KI, int KD, int I_max, unsigned char shift);
void pid_reset (PID_t *p);
int  pid_update(PID_t *p, int setPoint, int processValue);

#endif
