/*
 * state_stairs.cpp
 *
 *  Created on: 23 févr. 2015
 *      Author: guilhem
 */

#include "Arduino.h"
#include "state_types.h"
#include "state_stairs.h"
#include "lib_move.h"
#include "lib_attitude.h"
#include "lib_fan.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"

#define ANGLE_STAIRS_STARTED 11
#define ANGLE_CMD_STOP 10
#define TIME_RAG_RLSD 8000       //time to release carpet after stair climb begin(in ms)
#define TIME_STOP 3000           //time for stop after stairs climbed (in ms)
#define ANGLE_CARPET_HOLD 120
#define ANGLE_CARPET_RELEASE 40

#define TIME_RAG_RLSD_NO_ATTITUDE 6000
#define ANGLE_ON_FLOOR 126

int attitudeCmdStartStairs;
unsigned long timeStairsStarted = 0;

Servo servoCarpet;

void initHardStairs(int pin_servo){
	servoCarpet.attach(pin_servo);
	servoCarpet.write(ANGLE_CARPET_HOLD);
}

sState *testStairs()
	{
#ifdef ATTITUDE
	static unsigned long timeStopSoon = 0;
	int attitudeCmdStairs = servoAttitude.read();

	//réglage du temps de début de la montée
#ifndef NO_ATTITUDE_BEFORE_STAIRS
	if (abs(attitudeCmdStairs-attitudeCmdStartStairs) > ANGLE_STAIRS_STARTED && timeStairsStarted == 0 ){
		timeStairsStarted = millis();
		Serial.println("Début montée");
	}
#endif

	if (timeStairsStarted != 0){	//si on a commencé a monter :
		if (millis() - timeStairsStarted > TIME_RAG_RLSD) {	//après TIME_RAG_RLSD on relache les tapis.
						servoCarpet.write(ANGLE_CARPET_RELEASE);
						Serial.println("Relachement tapis");
				}
		//quand on revient a l'angle de  départ +/- ANGLE_CMD_STOP on déclanche le timer pour l'arret
		if (abs(attitudeCmdStairs-attitudeCmdStartStairs) < ANGLE_CMD_STOP && timeStopSoon==0){
			timeStopSoon = millis();
			Serial.println("Stop soon");
		}
	}

	//arret TIME_STOP après
	if(timeStopSoon!=0 && millis() - timeStopSoon > TIME_STOP){
		Serial.println("STOP !!!");
		if (digitalRead(PIN_COLOR)==COLOR_RED)return &sTrajEndStairsGreen;
		else return &sTrajEndStairsYellow;
	}
	return 0;
#else
	static unsigned long timestart = millis();

	if (millis() - timestart > TIME_RAG_RLSD_NO_ATTITUDE) {
		servoCarpet.write(40);
	}

	if(millis()-timestart > 14000){
		if (digitalRead(PIN_COLOR)==COLOR_RED)return &sTrajEndStairsGreen;
				else return &sTrajEndStairsYellow;
	}
	return 0;
#endif
}

void initStairs(sState *prev)
	{
	fanSetCon(FAN_SPEED);
	attitudeCmdStartStairs = servoAttitude.read();
#ifdef NO_ATTITUDE_BEFORE_STAIRS
	attitudeCmdStartStairs = ANGLE_ON_FLOOR;
	timeStairsStarted = millis();
#endif
	move(5,0);
#ifndef ATTITUDE
	servoAttitude.write(attitudeCmdStartStairs-100);
#endif
	#ifdef DEBUG
		Serial.println("Starting stairs");
	#endif

}

void deinitStairs(sState *next)
	{
	fanSetCon(0);
	move(0,0);

	#ifdef DEBUG
		Serial.println("Stairs are climbed");
	#endif
}

sState sStairs={
		BIT(E_MOTOR) |BIT(E_ATTITUDE),
		&initStairs,
		&deinitStairs,
		&testStairs
	};
