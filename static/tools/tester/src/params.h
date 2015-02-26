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
//pins
//digital I/O

#define CODERINT 2
#define ENCODER1 4
#define ENCODER2 5
#define SELECT 6
#define RETOUR 7
#define PIN_PWM_SERVO 11
#define LCD1 3
#define LCD2 8
#define LCD3 9
#define LCD4 10
#define LCD5 12
#define LCD6 13
#define CODERSTATE A0

#define DUREE_BIG_STEPS 20

#define PRECISION_MICROS 5	//précision pour le mode micros
#define DELAY_BOUNCE 15
#define DEBUG
#define NOLCD


#endif /* DEFS_H_ */
