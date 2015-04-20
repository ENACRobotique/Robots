/*
 * state_Prestairs.cpp
 *
 *  Created on: 2015 mars 18
 *      Author: Guilhem
 */


#include "Arduino.h"
#include "state_Prestairs.h"
#include "Arduino.h"
#include "state_types.h"
#include "state_stairs.h"
#include "lib_move.h"
#include "lib_attitude.h"
#include "../tools.h"
#include "../params.h"

unsigned long timeStartPreStairs;
#ifdef ATTITUDE
#define TIME_PRESTAIRS 4000
#else
#define TIME_PRESTAIRS 2000
#endif

sState* testPrestairs()
    {
        if (millis()-timeStartPreStairs > TIME_PRESTAIRS){
        	return &sStairs;
        }
       return NULL;
    }

void initPrestairs(sState *prev)
    {
        timeStartPreStairs = millis();
        move(0,0);
    }

void deinitPrestairs(sState *next)
    {
    }

sState sPrestairs={
	BIT(E_ATTITUDE)|BIT(E_MOTOR),
        &initPrestairs,
        &deinitPrestairs,
        &testPrestairs
};

