/*
 * state_launch.cpp
 *
 *  Created on: november 2013
 *      Author: Sebastien
 */


#include "Arduino.h"
#include "state_traj.h"
#include "state_pause.h"
#include "../params.h"
#include "../tools.h"

#include "lib_move.h"
#include "lib_radar2.h"


sState* testFresco()
	{
    if (radarIntrusion()) return &sPause;
    return 0;
	}

void initFresco(sState *prev)
	{
	#ifdef DEBUG
		Serial.println("Début fresco");
	#endif
	}

void deinitFresco(sState *next)
	{
	#ifdef DEBUG
		Serial.println("Fin fresco");
	#endif
	}

sState sFresco={
        BIT(E_MOTOR) | BIT(E_LINE),
        &initFresco,
        &deinitFresco,
        &testFresco
};

