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

Servo canne_servo;
Servo crema_servo;

sState* testPeche(){


#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	trajElem purple_fishing[] = {
			{200,0,1300},
			{100,0,7800},
			{0,0,100},
			{-300,-2,2300},
			{-300,0,1500},
			{-300,8,1150},
			{0,0,500},
			{0,0,100000},
	};
	static unsigned long st_saveTime=0;
	static int i=0;
	static unsigned long prev_millis=0;
	if (i==1){
		canne_servo.write(CANNE_DOWN);
	}
	if (i==3) {
		canne_servo.write(CANNE_UP);
	}
	if (i==6) {
		canne_servo.write(CANNE_DOWN);

	}
	if (i==7) {
		crema_servo.write(CREMA_IN);
	}
	if(periodicProgTraj(purple_fishing,&st_saveTime,&i,&prev_millis))
	{
		return &sWait;
	}

	return NULL;
}

void initPeche(sState *prev){
	move(0,0);

	canne_servo.attach(PIN_CANNE_A_PECHE);
	canne_servo.write(CANNE_UP);

	crema_servo.attach(PIN_CREMA);
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

