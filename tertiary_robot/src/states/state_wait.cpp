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



void initWait(sState *prev)
	{
    emergencyStop();
		#ifdef DEBUG
			Serial.println("start wait");
		#endif
	}

sState* testWait(){
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sDead;
    return 0;
}

sState sWait={
        BIT(E_MOTOR),
        &initWait,
        NULL,
        &testWait
    };


