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
#include "state_pause.h"

unsigned long st_saveTime_P=0,st_prevSaveTime_P=0,st_saveTime_radar_P=0,st_prevSaveTime_radar_P=0;

void initPechePurple(sState *prev){
	#ifdef DEBUG
	Serial.println(F("J'entre en etat peche"));
	#endif
	move(0,0);
	if (prev==&sPause)
	{
		#ifdef DEBUG
			Serial.println(F("\tback from pause"));
		#endif
		st_saveTime_P=millis()-st_saveTime_P+st_prevSaveTime_P;
		st_saveTime_radar_P=millis()-st_saveTime_radar_P+st_prevSaveTime_radar_P;
		_backFromPause = 1;
	}

	uint16_t limits[RAD_NB_PTS]={25,0,25,0};
	radarSetLim(limits);

	canne_servo.write(CANNE_UP);
	crema_servo.write(CREMA_OUT);
    }

void deinitPechePurple(sState *next){
	#ifdef DEBUG
	Serial.println(F("Je sors de l'etat peche"));
	#endif
	if (next==&sPause)
	{
		st_prevSaveTime_P=st_saveTime_P;
		st_saveTime_P=millis();
		st_prevSaveTime_radar_P=st_saveTime_radar_P;
		st_saveTime_radar_P=millis();
	}
	else
	{
		st_saveTime_P=0;
		st_prevSaveTime_P=0;
		st_saveTime_radar_P=0;
		st_prevSaveTime_radar_P=0;
	}
}

sState* testPechePurple();
sState sPeche2Purple={
	BIT(E_MOTOR)|BIT(E_RADAR),
	&initPechePurple,
	&deinitPechePurple,
	&testPechePurple
};
sState sPechePurple={
	BIT(E_MOTOR)|BIT(E_RADAR),
	&initPechePurple,
	&deinitPechePurple,
	&testPechePurple
};
const PROGMEM trajElem purple_fishing[] = {
	{0,0,400},
	{120,0,3225},//canne down	1
	{0,0,60},//				2
	{-200,-12,1000},//canne up
	{-200,-20,1500},
	{-200,35,2200},
	{-200,-15,2200},
	{0,0,500},//canne down		7
	{0,0,1000},//crema in		8
	{0,0,200},//canne up		9
	{0,0,100},//crema out		10
	{200,-15,2000},
	{200,35,1300},
	{200,-10,1200},
	{200,-2,800},//canne down	14
	{60,0,7000},
	{0,0,60},//canne up		16
	{-200,0,800},
	{-200,-2,500},
	{-200,-10,1700},
	{-200,30,2000},
	{-200,-15,2000},
	{0,0,500},//canne down		22
	{0,0,1000},//crema in		23
	{0,0,200},//canne up		24
	{0,0,100},//crema out		25

	{200,-15,2000},
	{200,35,1300},
	{200,0,1200},
	{200,-2,800},//canne down	29
	{60,0,7000},
	{0,0,60},//canne up		31
	{-200,0,800},
	{-200,-2,500},
	{-200,-10,1700},
	{-200,30,2000},
	{-200,-15,2000},
	{0,0,500},//canne down		37
	{0,0,1000},//crema in		38
	{0,0,200},//canne up		39
	{0,0,100},//crema out		40

//	{200,-15,2000},
//	{200,35,1700},
//	{200,-15,1200},
//	{200,-2,1400},
//
//	{0,30,100},// canne down 30
//	{-200,30,800},
//	{-200,-30,600},

	{0,0,5000},// et on recommence 33
	{0,0,0},
};
sState* testPechePurple(){


#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;

#endif


	static int i=0;
	static unsigned long prev_millis=0;
	static int pos_servo = CANNE_VERTICAL;
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION-1000)
	{
		canne_servo.write(CANNE_SECURE);
	}
	else
	{
		switch (i) {
			case 1:
				canne_servo.write(CANNE_DOWN);
				pos_servo = CANNE_DOWN;
				break;
			case 2:
				if ((millis()-prev_millis)>50 and pos_servo>CANNE_UP){
					pos_servo = pos_servo - 2;
					canne_servo.write(pos_servo);
					prev_millis=millis();
				}
				break;
			case 7:
				canne_servo.write(CANNE_DOWN);
				break;
			case 8:
				crema_servo.write(CREMA_IN);
				break;
			case 9:
				canne_servo.write(CANNE_UP);
				break;
			case 10:
				crema_servo.write(CREMA_OUT);
				break;
			case 14:
				canne_servo.write(CANNE_DOWN);
				pos_servo = CANNE_DOWN;
				break;
			case 16:
				if ((millis()-prev_millis)>50 and pos_servo>CANNE_UP){
					pos_servo = pos_servo - 2;
					canne_servo.write(pos_servo);
					prev_millis=millis();
				}
				break;
			case 22:
				canne_servo.write(CANNE_DOWN);
				break;
			case 23:
				crema_servo.write(CREMA_IN);
				break;
			case 24:
				canne_servo.write(CANNE_UP);
				break;
			case 25:
				crema_servo.write(CREMA_OUT);
				break;
			case 29:
				canne_servo.write(CANNE_DOWN);
				pos_servo = CANNE_DOWN;
				break;
			case 31:
				if ((millis()-prev_millis)>50 and pos_servo>CANNE_UP){
					pos_servo = pos_servo - 2;
					canne_servo.write(pos_servo);
					prev_millis=millis();
				}
				break;
			case 37:
				canne_servo.write(CANNE_DOWN);
				break;
			case 38:
				crema_servo.write(CREMA_IN);
				break;
			case 39:
				canne_servo.write(CANNE_UP);
				break;
			case 40:
				crema_servo.write(CREMA_OUT);
				break;
		}
	}
	if(periodicProgTraj(purple_fishing,&st_saveTime_P,&i,&prev_millis))
	{
		return &sWait;
	}
	if (radarIntrusion())
	{
		return &sPause;
	}
	return NULL;
}
///////////////////////////////////////////////////////////////////////////////
//Green

void initPecheGreen(sState *prev){
	Serial.println(F("J'entre en etat peche"));
	move(0,0);
	if (prev==&sPause)
	{
		#ifdef DEBUG
			Serial.println(F("\tback from pause"));
		#endif
		st_saveTime_P=millis()-st_saveTime_P+st_prevSaveTime_P;
		st_saveTime_radar_P=millis()-st_saveTime_radar_P+st_prevSaveTime_radar_P;
		_backFromPause = 1;
	}
	canne_servo.write(CANNE_UP);

	crema_servo.write(CREMA_OUT);

    uint16_t limits[RAD_NB_PTS]={25,0,25,0};
   	radarSetLim(limits);
    }

void deinitPecheGreen(sState *next){
	#ifdef DEBUG
	Serial.println(F("Je sors de l'etat peche"));
	#endif
	if (next==&sPause)
	{
		st_prevSaveTime_P=st_saveTime_P;
		st_saveTime_P=millis();
		st_prevSaveTime_radar_P=st_saveTime_radar_P;
		st_saveTime_radar_P=millis();
	}
	else
	{
		st_saveTime_P=0;
		st_prevSaveTime_P=0;
		st_saveTime_radar_P=0;
		st_prevSaveTime_radar_P=0;
	}
}

sState* testPecheGreen();
sState sPeche2Green={
	BIT(E_MOTOR)|BIT(E_RADAR),
	&initPecheGreen,
	&deinitPecheGreen,
	&testPecheGreen
};
sState sPecheGreen={
	BIT(E_MOTOR)|BIT(E_RADAR),
	&initPecheGreen,
	&deinitPecheGreen,
	&testPecheGreen
};

const PROGMEM trajElem green_fishing[] = {
	{0,0,100},
	{300,-1,1650},
	{100,0,4283},//canne down	2
	{0,0,60},//				3
	{200,-12,500},//canne up
	{200,-20,1500},
	{200,35,2000},
	{200,-15,2200},
	{0,0,500},//canne down		8
	{0,0,1000},//crema in		9
	{0,0,200},//canne up		10
	{0,0,100},//crema out		11
	{-200,-15,2000},
	{-200,35,1700},
	{-200,-15,1300},
	{-200,-3,1700},
	{100,0,4000},//canne down	16
	{0,0,60},//canne up 		17
	{-200,0,1100},
	{200,-15,1850},
	{200,35,1700},

	{200,-15,2000},
	{0,0,500},//canne down		22
	{0,0,1000},//crema in		23
	{0,0,200},//canne up		24
	{0,0,100},//crema out		25
	{-200,-15,2000},
	{-200,35,1700},
	{-200,-15,1300},
	{-200,-3,1700},
	{-300,0,2000},
	{-200,0,5000},// et on recommence 31
	{0,0,0},
};

sState* testPecheGreen(){

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	static int i=0;
	static unsigned long prev_millis=0;
	static int pos_servo = CANNE_VERTICAL;
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION-1000)
	{
		canne_servo.write(CANNE_SECURE);
	}
	else
	{
		switch (i) {
			case 2:
				canne_servo.write(CANNE_DOWN);
				pos_servo = CANNE_DOWN;
				break;
			case 3:
				if ((millis()-prev_millis)>50 and pos_servo>CANNE_UP){
					pos_servo = pos_servo - 2;
					canne_servo.write(pos_servo);
					prev_millis=millis();
				}
				break;
			case 8:
				canne_servo.write(CANNE_DOWN);
				break;
			case 9:
				crema_servo.write(CREMA_IN);
				break;
			case 10:
				canne_servo.write(CANNE_UP);
				break;
			case 11:
				crema_servo.write(CREMA_OUT);
				break;
			case 16:
				canne_servo.write(CANNE_DOWN);
				pos_servo = CANNE_DOWN;
				break;
			case 17:
				if ((millis()-prev_millis)>50 and pos_servo>CANNE_UP){
					pos_servo = pos_servo - 2;
					canne_servo.write(pos_servo);
					prev_millis=millis();
				}
				break;
			case 22:
				canne_servo.write(CANNE_DOWN);
				break;
			case 23:
				crema_servo.write(CREMA_IN);
				break;
			case 24:
				canne_servo.write(CANNE_UP);
				break;
			case 25:
				crema_servo.write(CREMA_OUT);
				break;
			case 31:
				if( digitalRead(PIN_SWITCH_LEFT) )
				{
					st_saveTime_P=0;
					i=0;
					prev_millis=0;
					pos_servo = CANNE_VERTICAL;
					return &sPeche2Green;
				}
				break;
		}
	}
	if(periodicProgTraj(green_fishing,&st_saveTime_P,&i,&prev_millis))
	{
		return &sWait;
	}
	if (radarIntrusion())
	{
		return &sPause;
	}
	return NULL;
}
