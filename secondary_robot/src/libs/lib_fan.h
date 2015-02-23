/*
 * lib_fan.h
 *
 *  Created on: 6 févr. 2015
 *      Author: guilhem
 */

#ifndef LIB_FAN_H_
#define LIB_FAN_H_

#include "Arduino.h"
#include "../params.h"

void fanInitHard(int pinFan);
void fanSetCon(int fanSpeed);

#endif /* SRC_LIBS_LIB_FAN_H_ */
