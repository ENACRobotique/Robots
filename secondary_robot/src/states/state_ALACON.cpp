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
#include "lib_fan.h"

#include "lib_move.h"



sState* testAlacon()
	{
		//do nothing
		return NULL;
	}

void initAlacon(sState *prev)
	{
	fanSetCon(254);
#ifdef DEBUG
    Serial.println("j'entre en A LA CON");
#endif


	}

void deinitAlacon(sState *next)
	{
	#ifdef DEBUG
		Serial.println("fin A LA CON");
	#endif
	}

sState sAlacon={
		BIT(E_ATTITUDE),
        &initAlacon,
        &deinitAlacon,
        &testAlacon
};

