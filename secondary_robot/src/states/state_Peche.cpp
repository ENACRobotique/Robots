/*
 * state_Peche.cpp
 *
 *  Created on: 2016 février 05
 *      Author: Darian
 */


#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_wait.h"
#include "lib_radar_mask.h"
#include "state_funny_action.h"

sState* testPeche(){

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	trajElem go_river[] = {

			{0,0,0}
	};
	static unsigned long st_saveTime=0;
	static int i=0;
	static unsigned long prev_millis=0;
	if(periodicProgTraj(go_river,&st_saveTime,&i,&prev_millis))
	{
		return &sWait;
	}
	return NULL;
}

void initPeche(sState *prev){


    }

void deinitPeche(sState *next){
        // Your code here !
    }

sState sPeche={
	BIT(E_MOTOR),
        &initPeche,
        &deinitPeche,
        &testPeche
};

