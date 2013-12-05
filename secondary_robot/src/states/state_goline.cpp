/*
 * state_goline.cpp
 *
 *  Created on: novembre 2013
 *      Author: Sebastien
 */


#include "Arduino.h"
#include "state_tirette.h"
#include "state_launch.h"
#include "state_pause.h"
#include "../params.h"
#include "../tools.h"

#include "lib_move.h"
#include "lib_line.h"


sState* testGolineRed()
	{
	static unsigned long prevIn=0;
	if (getIntensity(4)<4500) prevIn=millis();
	if (((millis() - prevIn) > DEBOUNCE_DELAY) && (getIntensity(4)>4500)) //Protection 1 fausse valeur
		{
		return &sLaunch;
		}
	if (EnemyDetection()) return &sPause;
	move(100,0); //A afininer avec trajectoire programmé
    return 0;
	}

void initGolineRed(sState *prev)
	{

	}

void deinitGolineRed(sState *next)
	{

	#ifdef DEBUG
		Serial.println("fin goline");
	#endif
	}

sState sGolineRed={
        BIT(E_MOTOR),
        &initGolineRed,
        &deinitGolineRed,
        &testGolineRed
};




sState* testGolineYellow()
	{
	static unsigned long prevIn=0;
	if (getIntensity(4)>4500) prevIn=millis();
	if ( (millis() - prevIn) > DEBOUNCE_DELAY)
		{
		return &sLaunch;
		}
	if (EnemyDetection()) return &sPause;
	move(100,0); //A afininer avec trajectoire programmé
    return 0;
	}

void initGolineYellow(sState *prev)
	{

	}

void deinitGolineYellow(sState *next)
	{

	#ifdef DEBUG
		Serial.println("fin goline");
	#endif
	}

sState sGolineYellow={
        BIT(E_MOTOR),
        &initGolineYellow,
        &deinitGolineYellow,
        &testGolineYellow
};
