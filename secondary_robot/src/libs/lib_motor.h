/*
 * lib_motor.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef LIB_MOTOR_H_
#define LIB_MOTOR_H_

#include "Arduino.h"
#include "lib_odo.h"
#include "params.h"

#define DEBUG_MOTOR

void motorInitHard(int pinDir[],int pinPWM[]);
void motAsser();
void motAsserTemp();

extern int _motPinPWM[NB_MOTORS];
extern int _motCon[NB_MOTORS];
inline void motSetCon(int motSpeed[]){
	for(int i=0;i<NB_MOTORS;i++)
	{
		_motCon[i]=motSpeed[i];
	}
}
#endif /* LIB_MOTOR_H_ */
