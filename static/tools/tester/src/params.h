/*
 * defs.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef DEFS_H_
#define DEFS_H_

/* includes ************************************************/
#include "Servo.h"

/* endincludes *********************************************/


/* Defines ************************************************/
//pins
//digital I/O
//#define PIN_RAD_SERVO 4
//#define PIN_DIR_SERVO 5
//#define PIN_MOTOR_PWM 6
//#define PIN_MOTOR_DIR 7
//#define PIN_FUNNY 10
//#define PIN_ODO_INT 2//odometer, interruption pin
//#define PIN_ODO_SEN 9
//#define PIN_TIRETTE 12
#define PIN_LED 13
#define SELECT 5
#define RETOUR 2
//#define PIN_ARM_LEFT 11
//#define PIN_ARM_RIGHT 3
#define PIN_COLOR 8

//analog in
#define PIN_SHARP_FRONT A0
#define PIN_SHARP_LEFT A2
#define PIN_SHARP_RIGHT A1

//times
#define TIME_MATCH_STOP 90000   //in ms
#define TIME_FUNNY_STOP 100000

//tirette related infos
#define TIRETTE_IN HIGH
#define DEBOUNCE_DELAY 100 //(in ms)

#define WALL_DST 23 //in cm<<1
#define WALL_SPEED 100

#define ANGLE_ZERO      30
#define DIR_SERVO_START -30// rem : en tenant compte de ANGLE_ZERO

#define COLOR_RED HIGH
#define COLOR_BLUE LOW

#define ARM_LEFT_UP 0
#define ARM_LEFT_DOWN 170
#define ARM_RIGHT_UP 162
#define ARM_RIGHT_DOWN 5
#define ARM_RIGHT_GLASS 35
#define ARM_RAISE_TIME 3000//in millis
#define ARM_2WAY_FAST 500   //in millis

#define UPPERCUT

#define RADAR_SAFETY_TIME 500//in ms
#define RADAR_SAFETY_DST 20 //in cm

#define DEBUG


/* endDefines *********************************************/





/*  externs*************************************************/
extern unsigned long _matchStart;
extern Servo armServoLeft,armServoRight;
extern int retour;
/*  end externs*********************************************/


#endif /* DEFS_H_ */
