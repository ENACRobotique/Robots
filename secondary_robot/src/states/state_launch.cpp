/*
 * state_launch.cpp
 *
 *  Created on: november 2013
 *      Author: Sebastien
 */


#include "Arduino.h"
#include "state_tirette.h"
#include "state_goline.h"
#include "state_pause.h"
#include "../params.h"
#include "../tools.h"

#include "lib_move.h"


sState* testLaunch()
	{
	if (EnemyDetection()) return &sPause;
    return 0;
	}

void initLaunch(sState *prev)
	{
	#ifdef DEBUG
		Serial.println("Début launch");
	#endif
	}

void deinitLaunch(sState *next)
	{
	#ifdef DEBUG
		Serial.println("Fin launch");
	#endif
	}

sState sLaunch={
        BIT(E_MOTOR) | BIT(E_LINE),
        &initLaunch,
        &deinitLaunch,
        &testLaunch
};

