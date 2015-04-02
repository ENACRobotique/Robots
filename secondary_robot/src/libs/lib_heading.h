/*
 * lib_heading.h
 *
 *  Created on: 03 fev. 2015
 *      Author: Fabien&guilhem
 */

#ifndef LIB_HEADING_H_
#define LIB_HEADING_H_

#include "Arduino.h"
#include "../params.h"
#include "Servo.h"
#include "Wire.h"
#include "MPU_6050.h"
//#define DEBUG_ATTITUDE

#define X_ANGLE 1
#define Y_ANGLE 2
#define Z_ANGLE 3
#define X_ACCEL 11
#define Y_ACCEL 12
#define Z_ACCEL 13
#define X_ANGLE_ACCEL 21
#define Y_ANGLE_ACCEL 22
#define Z_ANGLE_ACCEL 23

extern int _headingCon;
extern int _OmegaCmd;
extern int _speedCon;

void headingAsser();
int headingGetCurrent();

inline void headingSetCon(int teta){
	_headingCon = teta;
}

inline void speedSetCon(int speed){
	_speedCon = speed;
}

inline int headingGetCon(){
	return _headingCon;
}

#endif
