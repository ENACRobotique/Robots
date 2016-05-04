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
#include "state_funny_action.h"
#include "state_pause.h"
#include "state_Peche.h"

int purple=0;
unsigned long st_saveTime_R=0,st_prevSaveTime_R=0,st_saveTime_radar_R=0,st_prevSaveTime_radar_R=0;


const PROGMEM trajElem calage_largeur_purple[] = {
	{300,0,1400},
	{0,-45,200},
	{-200,-45,2450},
	{0,0,0},
};

const PROGMEM trajElem calage_largeur_green[] = {
	{0,60,300},
	{250,60,1700},
	{0,0,100},
	{0,0,0}
};

sState* testRecalage(){

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	uint16_t limits[RAD_NB_PTS]={25,0,0,0};

	const trajElem* calage=calage_largeur_green;

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
		move(-300,0);
		radarSetLim(limits);
		if(digitalRead(PIN_SWITCH_LEFT))
			{
				move(0,0);
				st_saveTime = 0;
				i = 0;
				flag_end = 0;
				if (digitalRead(PIN_COLOR)==COLOR_GREEN)return &sPecheGreen;
				else return &sPechePurple;

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
	if (prev==&sPause)
	{
		#ifdef DEBUG
			Serial.println(F("\tback from pause"));
		#endif
		st_saveTime_R=millis()-st_saveTime_R+st_prevSaveTime_R;
		st_saveTime_radar_R=millis()-st_saveTime_radar_R+st_prevSaveTime_radar_R;
		_backFromPause = 1;
	}
	uint16_t limits[RAD_NB_PTS]={0,0,0,25};
	radarSetLim(limits);
}

void deinitRecalage(sState *next){
	if (next==&sPause)
	{
		st_prevSaveTime_R=st_saveTime_R;
		st_saveTime_R=millis();
		st_prevSaveTime_radar_R=st_saveTime_radar_R;
		st_saveTime_radar_R=millis();
	}
	else
	{
		st_saveTime_R=0;
		st_prevSaveTime_R=0;
		st_saveTime_radar_R=0;
		st_prevSaveTime_radar_R=0;
	}
}

sState sRecalage={
	BIT(E_MOTOR)|BIT(E_RADAR),
    &initRecalage,
    &deinitRecalage,
    &testRecalage
};

