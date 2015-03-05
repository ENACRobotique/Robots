/*
 * state_ALACON.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */


#include "Arduino.h"
#include "state_ALACON.h"
#include "../params.h"
#include "../tools.h"

#include "lib_move.h"



sState* testAlacon()
	{
		//do nothing
		return NULL;
	}

void initAlacon(sState *prev)
	{
#ifdef DEBUG
    Serial.println("j'entre en A LA CON");
    move(10,10);
#endif


	}

void deinitAlacon(sState *next)
	{
    _matchStart=millis();

	#ifdef DEBUG
		Serial.println("fin A LA CON");
	#endif
	}

sState sAlacon={
		BIT(E_MOTOR),
        &initAlacon,
        &deinitAlacon,
        &testAlacon
};

