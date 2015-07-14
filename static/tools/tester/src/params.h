/*
 * defs.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef DEFS_H_
#define DEFS_H_

/* includes ************************************************/

#include <Servo.h>

/* Defines ************************************************/
#define DEBUG
//#define NOLCD

//pins
//digital I/O

#define SERVO1 0
#define SERVO2 1
#define PIN_PWM_SERVO 11

#define CODERINT 2
#define CODERSTATE 4

#define ENCODER1 3
#define ENCODER2 5
#define SELECT 6
#define RETOUR A2

#define LCD1 7
#define LCD2 8
#define LCD3 9
#define LCD4 10
#define LCD5 12
#define LCD6 13

#define AREAD A0
#define LED1 A1
#define LED2 A3


#define DUREE_BIG_STEPS 20
#define PRECISION_MICROS 5	//précision pour le mode micros
#define DELAY_BOUNCE 15


#endif /* DEFS_H_ */
