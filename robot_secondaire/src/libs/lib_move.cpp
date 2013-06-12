/*
 * lib_move.cpp
 *
 *  Created on: 25 avr. 2013
 *      Author: quentin
 */
#include "Arduino.h"
#include "Servo.h"
#include "lib_move.h"

//globals
int _servo_zero=0;
Servo _dirServo;

//macros
#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//define
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

//sets speed and servo dir (angle=0 means straight line)
void move(int speed,int angle){
    motSetCon(speed);
    _dirServo.write(CLAMP(MIN_ANGLE , angle+_servo_zero, MAX_ANGLE));
}



