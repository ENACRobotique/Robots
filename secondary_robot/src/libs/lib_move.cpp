/*
 * lib_move.cpp
 *
 *  Created on: 25 avr. 2013
 *      Author: quentin
 */
#include "Arduino.h"
#include "Servo.h"
#include "lib_move.h"

#if defined(TANK) && defined(TRIKE)
#error "TANK and TRIKE defined. Only one possible."
#endif

//macros
#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

#define DERIVE 1

#ifdef TRIKE
Servo _dirServo;
int _servo_zero=0;

#define MAX_ANGLE 180
#define MIN_ANGLE 0

//initialises stirring servo (1pin + "null value"+ First value)
//REQUIRES : motorInitHard()
void moveInitHard(int pinDirServo,int zeroAngle,int startAngle){
    pinMode(pinDirServo,OUTPUT);
    _servo_zero=zeroAngle;
    _dirServo.attach(pinDirServo);
    _dirServo.write(CLAMP(MIN_ANGLE , startAngle+_servo_zero, MAX_ANGLE));
}
#endif

//sets speed motors (omega=0 means straight line)
void move(int speed,int omega){
#ifdef TANK
	int derive = 0;
	if (speed || omega)
	{
		derive = DERIVE;
	}
	int speeds[NB_MOTORS]={-speed - omega, -speed + omega + derive};
    motSetCon(speeds);
#else
#ifdef TRIKE
    motSetCon({speed});
    _dirServo.write(CLAMP(MIN_ANGLE , angle+_servo_zero, MAX_ANGLE));
#endif
#endif
}



