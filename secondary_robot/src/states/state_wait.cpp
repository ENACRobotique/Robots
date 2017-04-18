/*
 * state_wait.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "Arduino.h"
#include "../params.h"
#include "../tools.h"
#include "state_wait.h"
#include "state_dead.h"
#include "lib_move.h"
#include "state_funny_action.h"


void initWait(sState *prev)
	{
    emergencyStop();
		#ifdef DEBUG
			Serial.println("start wait");
		#endif
	}

sState* testWait(){
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sDead;

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

    return 0;
}

sState sWait={
        BIT(E_MOTOR),
        &initWait,
        NULL,
        &testWait
    };


