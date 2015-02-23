/*
 * lib_fan.cpp
 *
 *  Created on: 6 févr. 2015
 *      Author: guilhem
 */

#include "lib_fan.h"

#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

int _fanPin;

void fanInitHard(int pinFan){
	_fanPin = pinFan;
	pinMode(pinFan, OUTPUT);
	analogWrite(pinFan, 0);
}

void fanSetCon(int fanSpeed){
	analogWrite(_fanPin, CLAMP(0, abs(fanSpeed), 254));
}

