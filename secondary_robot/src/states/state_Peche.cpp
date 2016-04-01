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
#include "state_tirette.h"

sState* testPeche(){


#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	trajElem purple_fishing[] = {
			{200,0,1300},
			{90,0,8900},
			{0,0,100},//Fish taken
			{-300,-2,2300},
			{-300,0,1500},
			{-300,8,2000},
			{0,0,500},
			{0,0,50000},//Fish drop
			{300,2,2000},
			{90,0,8900},
			{0,0,100000},
	};
	static unsigned long st_saveTime=0;
	static int i=0;
	static unsigned long prev_millis=0;
	static int pos_servo = CANNE_VERTICAL;
	switch (i) {
		case 1:
			canne_servo.write(CANNE_DOWN);
			pos_servo = CANNE_DOWN;
			break;
		case 3:
			if ((millis()-prev_millis)>300 and pos_servo>CANNE_UP){
				pos_servo = max(pos_servo - 5, CANNE_UP);
				canne_servo.write(pos_servo);
			}
			break;
		case 6:
			canne_servo.write(CANNE_DOWN);
			break;
		case 7:
			crema_servo.write(CREMA_IN);
			break;
	}
	if(periodicProgTraj(purple_fishing,&st_saveTime,&i,&prev_millis))
	{
		return &sWait;
	}

	return NULL;
}

void initPeche(sState *prev){
	move(0,0);

	canne_servo.write(CANNE_UP);

	crema_servo.write(CREMA_OUT);
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

