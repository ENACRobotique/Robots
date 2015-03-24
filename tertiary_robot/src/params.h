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
#define PIN_DIR_SERVO 5
#define PIN_MOTOR_PWM 6
#define PIN_MOTOR_DIR 7
#define PIN_ODO_INT 2//odometer, interruption pin
#define PIN_ODO_SEN 9
#define PIN_TIRETTE 10
#define PIN_LED 13
#define PIN_LAUNCHER_1 8
#define PIN_LAUNCHER_2 3
#define PIN_LAUNCHER_NET 4
#define PIN_COLOR 12

//analog in
#define PIN_SHARP_FRONT_RIGHT A0
#define PIN_SHARP_FRONT_LEFT A1
#define PIN_SHARP_BACK_RIGHT A2
#define PIN_SHARP_BACK_LEFT A3

//times
#define TIME_MATCH_STOP 90000   //in ms
#define TIME_FUNNY_STOP 95000
#define TIME_MATCH_LAUN 85000

//tirette related infos
#define TIRETTE_IN HIGH
#define DEBOUNCE_DELAY 100 //(in ms)

#define WALL_DST 15 //in cm<<1
#define WALL_SPEED 80

#define ANGLE_ZERO      30
#define DIR_SERVO_START 0// rem : en tenant compte de ANGLE_ZERO

#define COLOR_RED HIGH
#define COLOR_YELLOW LOW


#define UPPERCUT

#define RADAR_SAFETY_DST 30 //in cm
#define RADAR_SAFETY_TIME 500 //in ms

#define DEBUG

/* endDefines *********************************************/





/*  externs*************************************************/
extern unsigned long _matchStart;
extern Servo launcherServoUp,launcherServoDown, launcherServoNet;

/*  end externs*********************************************/


#endif /* DEFS_H_ */
