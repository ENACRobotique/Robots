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

//#define DEBUG_MOTOR

void motorInitHard(int pinDir,int pinPWM);
void motAsser();
void motAsserTemp();

extern int _motCon;
inline void motSetCon(int motSpeed){
    _motCon=motSpeed;
}
#endif /* LIB_MOTOR_H_ */
