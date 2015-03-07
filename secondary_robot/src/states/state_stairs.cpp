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



int attitudeCmdStartStairs;
unsigned long timeStairsStarted = 0;
int rangeAttitudeCmdStop = -1;
Servo servoCarpet;

void initHardStairs(int pin_servo){
	servoCarpet.attach(pin_servo);
	servoCarpet.write(110);
}

sState *testStairs()
	{
	static int timeForArtefact = 0;
	int attitudeCmdStairs = attitudeGetCmd();
	if (90+attitudeCmdStairs <= 160 && timeStairsStarted == 0 && timeForArtefact == 0){
		timeForArtefact = millis();
#ifdef DEBUG
		Serial.println(timeForArtefact);
#endif
	}
	if (90+attitudeCmdStairs > 160 && timeStairsStarted == 0){
			timeForArtefact = 0;
		}
	if (millis() - timeForArtefact > 1000 && timeForArtefact != 0 && timeStairsStarted == 0)
	{
		timeStairsStarted = millis();
	}

	if (timeStairsStarted != 0){
		if (millis()-timeStairsStarted >= TIME_RAG_RLSD){
			servoCarpet.write(40);
	}
		if (/*(abs(attitudeCmdStairs-attitudeCmdStartStairs) <= rangeAttitudeCmdStop))||*/ millis()-timeStairsStarted > 6500) {
			if (digitalRead(PIN_COLOR)==COLOR_RED)return &sTrajEndStairsGreen;
			else return &sTrajEndStairsYellow;
		}
	}
	return 0;
}

void initStairs(sState *prev)
	{
	fanSetCon(FAN_SPEED);
	move(10,0);
	attitudeCmdStartStairs = attitudeGetCmd();
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
