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



int attitudeConStartStairs;
unsigned long timeStairsStarted;
int rangeAttitudeConStop = 10;

sState *testStairs()
	{
	if (millis()-timeStairsStarted >= TIME_RAG_RLSD){

	}
	int attitudeConStairs = _attitudeCon;
	if (attitudeConStairs <= attitudeConStartStairs + rangeAttitudeConStop && attitudeConStairs >= attitudeConStartStairs - rangeAttitudeConStop) {
		if (digitalRead(PIN_COLOR)==COLOR_RED)return &sTrajEndStairsGreen;
		else return &sTrajEndStairsYellow;
	}
	return 0;
}

void initStairs(sState *prev)
	{
	fanSetCon(FAN_SPEED);
	move(20,0);
	attitudeConStartStairs = _attitudeCon;
	timeStairsStarted = millis();
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
