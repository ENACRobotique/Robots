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


void initPechePurple(sState *prev){
	Serial.println(F("J'entre en etat peche"));
	move(0,0);

	canne_servo.write(CANNE_UP);

	crema_servo.write(CREMA_OUT);
    }

void deinitPechePurple(sState *next){
	Serial.println(F("Je sors de l'etat peche"));
    }

sState* testPechePurple();
sState sPeche2Purple={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
		&initPechePurple,
		&deinitPechePurple,
		&testPechePurple
};
sState sPechePurple={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initPechePurple,
        &deinitPechePurple,
        &testPechePurple
};

sState* testPechePurple(){


#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	trajElem purple_fishing[] = {
			{0,0,400},//				0
			{60,0,6450},//canne down
			{0,0,1000},//				2
			{-200,-12,500},//canne up
			{-200,-20,1500},
			{-200,35,2000},
			{-200,-15,2200},
			{0,0,500},//canne down		7
			{0,0,1000},//crema in		8
			{0,0,200},//canne up		9
			{0,0,100},//crema out		10
			{200,-15,2000},
			{200,35,1500},
			{200,-15,1200},
			{200,-2,800},//canne down	14
			{60,0,7000},
			{0,0,1000},//canne up		16
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
			{200,35,1700},
			{200,-15,1200},
			{200,-2,1400},

			{0,30,100},
			{-200,30,800},
			{-200,-30,600},

			{-200,0,5000},// et on recommence 33
			{0,0,0},
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
		case 2:
			if ((millis()-prev_millis)>500 and pos_servo>CANNE_UP){
				pos_servo = max(pos_servo - 3, CANNE_UP);
				canne_servo.write(pos_servo);
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
			if ((millis()-prev_millis)>500 and pos_servo>CANNE_UP){
				pos_servo = max(pos_servo - 3, CANNE_UP);
				canne_servo.write(pos_servo);
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
		case 33:
			if( digitalRead(PIN_SWITCH_LEFT) )
			{
				st_saveTime=0;
				i=0;
				prev_millis=0;
				pos_servo = CANNE_VERTICAL;
				return &sPeche2Purple;
			}
			break;
	}
	if(periodicProgTraj(purple_fishing,&st_saveTime,&i,&prev_millis))
	{
		return &sWait;
	}

	return NULL;
}
///////////////////////////////////////////////////////////////////////////////
//Green

void initPecheGreen(sState *prev){
	Serial.println(F("J'entre en etat peche"));
	move(0,0);

	canne_servo.write(CANNE_UP);

	crema_servo.write(CREMA_OUT);
    }

void deinitPecheGreen(sState *next){
	Serial.println(F("Je sors de l'etat peche"));
    }

sState* testPecheGreen();
sState sPeche2Green={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
		&initPecheGreen,
		&deinitPecheGreen,
		&testPecheGreen
};
sState sPecheGreen={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initPecheGreen,
        &deinitPecheGreen,
        &testPecheGreen
};

sState* testPecheGreen(){


#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	trajElem purple_fishing[] = {
			{0,0,400},//				0
			{60,0,6450},//canne down
			{0,0,1000},//				2
			{-200,-12,500},//canne up
			{-200,-20,1500},
			{-200,35,2000},
			{-200,-15,2200},
			{0,0,500},//canne down		7
			{0,0,1000},//crema in		8
			{0,0,200},//canne up		9
			{0,0,100},//crema out		10
			{200,-15,2000},
			{200,35,1500},
			{200,-15,1200},
			{200,-2,800},//canne down	14
			{60,0,7000},
			{0,0,1000},//canne up		16
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
			{200,35,1700},
			{200,-15,1200},
			{200,-2,1400},

			{0,30,100},
			{-200,30,800},
			{-200,-30,600},

			{-200,0,5000},// et on recommence 33
			{0,0,0},
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
		case 2:
			if ((millis()-prev_millis)>500 and pos_servo>CANNE_UP){
				pos_servo = max(pos_servo - 3, CANNE_UP);
				canne_servo.write(pos_servo);
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
			if ((millis()-prev_millis)>500 and pos_servo>CANNE_UP){
				pos_servo = max(pos_servo - 3, CANNE_UP);
				canne_servo.write(pos_servo);
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
		case 33:
			if( digitalRead(PIN_SWITCH_LEFT) )
			{
				st_saveTime=0;
				i=0;
				prev_millis=0;
				pos_servo = CANNE_VERTICAL;
				return &sPeche2Purple;
			}
			break;
	}
	if(periodicProgTraj(purple_fishing,&st_saveTime,&i,&prev_millis))
	{
		return &sWait;
	}

	return NULL;
}
