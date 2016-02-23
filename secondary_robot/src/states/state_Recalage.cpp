/*
 * state_Recalage.cpp
 *
 *  Created on: 2016 février 05
 *      Author: Darian
 */


#include "Arduino.h"
#include "state_Recalage.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_wait.h"
#include "lib_radar_mask.h"
#include "state_PrePeche.h"
#include "state_funny_action.h"
#include "state_pause.h"

int purple=0;

sState* testRecalage(){

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	trajElem calage_largeur_purple[] = {
					{0,60,300},
					{250,60,1650},
					{0,0,100},
					{0,0,0}
				};

	trajElem calage_largeur_green[] = {
						{0,-60,300},
						{250,-60,1650},
						{0,0,100},
						{0,0,0}
				};

	trajElem* calage=calage_largeur_green;

		if(purple)
		{
			calage = calage_largeur_purple;
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
			if(digitalRead(pin))
				{
					move(0,0);
					return &sPrePeche;
				}
		}
		if (radarIntrusion()) return &sPause;
		return NULL;
	}



void initRecalage(sState *prev){
	if (digitalRead(PIN_COLOR)==COLOR_GREEN){
		purple=0;
	}
	else{
		purple=1;
	}
}

void deinitRecalage(sState *next){
        // Your code here !
    }

sState sRecalage={
	BIT(E_MOTOR),
    &initRecalage,
    &deinitRecalage,
    &testRecalage
};

