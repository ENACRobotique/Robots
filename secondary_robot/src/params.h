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
#include <avr/pgmspace.h>
/* endincludes *********************************************/

//#define HEADING
//#define ATTITUDE

//#define NO_ATTITUDE_BEFORE_STAIRS

#define TIME_BEFORE_START 10
/* Defines ************************************************/
//pins
//digital I/O
#define TRIKE
#define PIN_SERVO_DIR 5
#define NB_MOTORS 1

#define PIN_MOTOR1_PWM 6
#define PIN_MOTOR1_DIR 7
#define PIN_ODO1_INT 2  //odometer1, interruption pin
#define PIN_ODO1_SEN 9

//POMPE
#define PIN_POMPE_PWM 11
#define PIN_POMPE_DIR A2

#define PIN_TIRETTE 10
#define PIN_COLOR 4


//times
#define TIME_MATCH_STOP 94000  //in ms  en théorie 90s mais on prend un peu de marge
#define TIME_FOR_FUNNY_ACTION 90000 // the beginning of the funny action

#define ANGLE_ZERO 98

//tirette related infos
#define TIRETTE_IN HIGH
#define DEBOUNCE_DELAY 100 //(in ms)

#define COLOR_BLUE LOW
#define COLOR_YELLOW HIGH


#define RADAR_SAFETY_DST 30 //in cm
#define RADAR_SAFETY_TIME 500 //in ms

#define TIME_READY_LAUNCHER 500 //in ms


#define HODOR 8
#define HODOR_CLOSE 130
#define HODOR_OPEN 30


#define DYN_USE
#ifdef  DYN_USE
#define DATA_DYNAMIXEL 3
#define NUM_DYNAMIXEL 1
#define DYN_UP 485
#define DYN_DOWN 825
#endif

#define DEBUG

/* endDefines *********************************************/





/*  externs*************************************************/
extern unsigned long _matchStart;

extern Servo Hodor;

/*  end externs*********************************************/


#endif /* DEFS_H_ */
