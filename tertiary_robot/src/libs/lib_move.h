/*
 * lib_move.h
 *
 *  Created on: 25 avr. 2013
 *      Author: quentin
 */

#ifndef LIB_MOVE_H_
#define LIB_MOVE_H_

#include "lib_motor.h"
#include "Arduino.h"

//initialises stirring servo (1pin + "null value"+ First value)
void moveInitHard(int pinDirServo,int zeroAngle,int startAngle);


void move(int speed,int angle);




#endif /* LIB_MOVE_H_ */
