/*
 * lib_attitude.h
 *
 *  Created on: 03 fev. 2015
 *      Author: Fabien
 */

#ifndef LIB_ATTITUDE_H_
#define LIB_ATTITUDE_H_

#include "Arduino.h"
#include "../params.h"
#include "Servo.h"
#include "Wire.h"
#define DEBUG_ATTITUDE


extern Servo servoAttitude;

void servoInitHard(int pinservo);
void attitudeAsser();
void attitudeAsserTemp();

extern int _attitudeCon;
inline void attitudeSetCon(int attitude){
	_attitudeCon=attitude;
}
#endif /* LIB_ATTITUDE_H_ */
