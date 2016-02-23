/*
 * state_PrePeche.cpp
 *
 *  Created on: 23 févr. 2016
 *      Author: liehn
 */



#include "Arduino.h"
#include "state_PrePeche.h"
#include "state_Recalage.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_wait.h"
#include "lib_radar_mask.h"
#include "state_Peche.h"
#include "state_funny_action.h"
#include "state_pause.h"


sState* testPrePeche(){

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	trajElem calage_purple[] = {
					{0,-20,300},
					{200,-20,2500},
					{0,-45,300},
					{-200,-45,3000},
					{-300,0,1600},
					{-300,-1,1600},
					{0,-30, 300},
					{-300,-30,2000},//Degagement palet
					{0,0,300},
					{-300,0,1800},
					{0,0,100},
					{300,-25,2000},
					{0,0,300},
					{300,0,2000},
					{0,-90,300},
					{300,-90,1200},
					{0,0,300},
					{0,0,0},
					//trajectoire prépeche
				};

	trajElem calage_green[] = {
						{0,10,300},
						{-200,10,500},
						{0,10,20000},
						//trajectoire prépeche
				};

	trajElem* calage=calage_green;

		if(purple)
		{
			calage = calage_purple;
		}
        static unsigned long st_saveTime=0;
        static int i=0;
		static unsigned long prev_millis=0;
		static int flag_end = 0;
		if(!flag_end){
			if(periodicProgTraj(calage,&st_saveTime,&i,&prev_millis))
				{
					flag_end = 1;
				}
		}
		else{
			int pin=PIN_SWITCH_RIGHT;
			if(purple){
				pin = PIN_SWITCH_LEFT;
			}
			move(-300,0);
			if(digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
				move(0,0);
				return &sRecalage;
			}
		}
		if (radarIntrusion()) return &sPause;
		return NULL;
	}



void initPrePeche(sState *prev){
	if (digitalRead(PIN_COLOR)==COLOR_GREEN){
		purple=0;
	}
	else{
		purple=1;
	}
}

void deinitPrePeche(sState *next){
        // Your code here !
    }

sState sPrePeche={
	BIT(E_MOTOR),
    &initPrePeche,
    &deinitPrePeche,
    &testPrePeche
};



