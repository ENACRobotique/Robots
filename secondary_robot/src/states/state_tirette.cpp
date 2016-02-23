/*
 * state_tirette.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */


#include "Arduino.h"
#include "state_tirette.h"
#include "state_traj.h"
#include "lib_radar.h"
#include "../params.h"
#include "../tools.h"
#include "lib_move.h"

/* State : tirette, first state of all, waits until the tirette is pulled
 *
 * tirette pulled -> next state, according to the "start side swich"
 *
 */

sState* testTirette()
	{
	_matchStart=millis();
    static unsigned long prevIn=0;  //last time the tirette was seen "in"
    static unsigned long timepull=0;
    if (digitalRead(PIN_TIRETTE)==TIRETTE_IN){
    	prevIn=millis();
    }
    if ( ( millis() - prevIn) > DEBOUNCE_DELAY)
    	{
    		if(timepull==0){timepull = millis();}
			if(millis() - timepull > TIME_BEFORE_START){
				if (digitalRead(PIN_COLOR)==COLOR_GREEN)return &sTrajGreenInit;
				else return &sTrajPurpleInit;
			}
    	}
    return 0;
	}

void initTirette(sState *prev)
	{
    move(0,0);
#ifdef DEBUG
    Serial.println("j'entre en tirette");
#endif


	}

void deinitTirette(sState *next)
	{
    _matchStart=millis();

	#ifdef DEBUG
		Serial.println("fin tirette");
	#endif
	}

sState sTirette={
        BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initTirette,
        &deinitTirette,
        &testTirette
};

