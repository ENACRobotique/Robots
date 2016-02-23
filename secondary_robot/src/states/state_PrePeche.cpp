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


sState* testPrePeche(){

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	trajElem calage_largeur_purple[] = {
					{0,-10,300},
					{-200,-10,500},
					{0,-10,20000},
					//trajectoire prépeche
				};

	trajElem calage_largeur_green[] = {
						{0,10,300},
						{-200,10,500},
						{0,10,20000},
						//trajectoire prépeche
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
					return &sPeche;
				}
		}
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



