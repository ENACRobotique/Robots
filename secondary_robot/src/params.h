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

#define HEADING
#define ATTITUDE

/* Defines ************************************************/
//pins
//digital I/O
#define TANK
#define NB_MOTORS 2
#define PIN_MOTOR1_PWM 5
#define PIN_MOTOR1_DIR 12
#define PIN_ODO1_INT 2//odometer1, interruption pin
#define PIN_ODO1_SEN 7
#define PIN_MOTOR2_PWM 6
#define PIN_MOTOR2_DIR 13
#define PIN_ODO2_INT 3//odometer2, interruption pin
#define PIN_ODO2_SEN 8
#define PIN_TIRETTE A0
//#define PIN_LED 13
#define PIN_COLOR 4
#define PIN_SERVO_ATTITUDE 10
#define PIN_SERVO_CARPET 9
#define PIN_VENTILO 11
//analog in
//#define PIN_SHARP_FRONT_RIGHT A0
//#define PIN_SHARP_FRONT_LEFT A1
//#define PIN_SHARP_BACK_RIGHT A2
//#define PIN_SHARP_BACK_LEFT A3

//times
#define TIME_MATCH_STOP 90000   //in ms
#define TIME_FUNNY_STOP 95000
#define TIME_MATCH_LAUN 85000

//tirette related infos
#define TIRETTE_IN HIGH
#define DEBOUNCE_DELAY 100 //(in ms)

#define WALL_DST 15 //in cm<<1
#define WALL_SPEED 80

#define COLOR_RED HIGH
#define COLOR_YELLOW LOW

#define UPPERCUT

#define RADAR_SAFETY_DST 30 //in cm
#define RADAR_SAFETY_TIME 500 //in ms

#define TIME_READY_LAUNCHER 500 //in ms

#define FAN_SPEED 254 //PWM Command(0-255)

#define DEBUG 1

/* endDefines *********************************************/





/*  externs*************************************************/
extern unsigned long _matchStart;
extern Servo launcherServoUp,launcherServoDown, launcherServoNet;

/*  end externs*********************************************/


#endif /* DEFS_H_ */
