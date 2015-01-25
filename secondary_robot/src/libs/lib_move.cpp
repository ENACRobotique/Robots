/*
 * lib_move.cpp
 *
 *  Created on: 25 avr. 2013
 *      Author: quentin
 */
#include "Arduino.h"
#include "Servo.h"
#include "lib_move.h"


//macros
#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif


//sets speed motors (omega=0 means straight line)
void move(int speed,int omega){
	int speeds[NB_MOTORS]={speed + omega, speed - omega};
    motSetCon(speeds);
}



