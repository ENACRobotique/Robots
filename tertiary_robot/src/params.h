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
#define TRIKE
#define NB_MOTORS 1
//#error "check pin attribution"
#define PIN_DIR_SERVO 5
#define PIN_MOTOR1_PWM 6
#define PIN_MOTOR1_DIR 7
#define PIN_ODO1_INT 2//odometer, interruption pin
#define PIN_ODO1_SEN 9
#define PIN_TIRETTE 10
#define PIN_LED 13
#define PIN_CLAP 3
#define PIN_COLOR 12
#define PIN_SHARP1 A1
#define PIN_SHARP2 A2
//times
#define TIME_MATCH_STOP 90000   //in ms

//tirette related infos
#define TIRETTE_IN HIGH
#define DEBOUNCE_DELAY 100 //(in ms)

#define ANGLE_ZERO      30
#define DIR_SERVO_START 0// rem : en tenant compte de ANGLE_ZERO

#define COLOR_GREEN LOW
#define COLOR_YELLOW HIGH

#define RADAR_SAFETY_DST 30 //in cm
#define RADAR_SAFETY_TIME 500 //in ms

#define CLAPNEUTRAL 100
#define CLAPYELLOW 176
#define CLAPGREEN 37

#define US_LOWEST_ADDR 0xE2
#define SHARP_ONLY
#define DEBUG

/* endDefines *********************************************/





/*  externs*************************************************/
extern unsigned long _matchStart;
extern Servo servoClap;

/*  end externs*********************************************/


#endif /* DEFS_H_ */
