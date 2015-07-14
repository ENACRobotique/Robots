/*
 * lib_heading.h
 *
 *  Created on: 03 fev. 2015
 *      Author: Fabien&guilhem
 */

#ifndef LIB_HEADING_H_
#define LIB_HEADING_H_

#include "Arduino.h"
#include "Servo.h"
#include "Wire.h"
#include "MPU_6050.h"
#include "state_types.h"
//#define DEBUG_HEADING

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
