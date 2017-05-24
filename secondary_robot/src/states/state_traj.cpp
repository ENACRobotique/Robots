
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_recup.h"
#include "state_pause.h"
#include "state_funny_action.h"
#include "state_wait.h"
#include "state_dead.h"

static unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
static periodicTraj periodicFunction = &periodicProgTraj;
#endif

static unsigned long pause_time =0;
static unsigned long start_pause=0;

void initTrajyellowInit(sState *prev)
{
#ifdef DEBUG
	Serial.println(F("debut traj (premier trajet)"));
#endif

	if (prev==&sPause)
	{
#ifdef DEBUG
		Serial.println(F("\tback from pause"));
#endif
		st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		st_saveTime_radar=millis()-st_saveTime_radar+st_prevSaveTime_radar;
		_backFromPause = 1;
		pause_time+=(millis()-start_pause);
	}
#ifdef DEBUG
	Serial.println(pause_time);
#endif
	//						   E 6 4  2  0
	uint16_t limits[RAD_NB_PTS]={0,0,30,30};
	radarSetLim(limits);
}

void deinitTrajyellowInit(sState *next)
{
	if (next==&sPause)
	{
		st_prevSaveTime=st_saveTime;
		st_saveTime=millis();
		st_prevSaveTime_radar=st_saveTime_radar;
		st_saveTime_radar=millis();
	}
	else
	{
		st_saveTime=0;
		st_prevSaveTime=0;
		st_saveTime_radar=0;
		st_prevSaveTime_radar=0;
		move(0,0);
		uint16_t limits[RAD_NB_PTS]={0,0,0,0};
		radarSetLim(limits);
	}
}

const PROGMEM trajElem start_blue[]={
		//Début trajectoire blue
		{-200,0,23,DISTANCE},
		QUART_TOUR_POS, //compte pour 3 instructions
		{0,0,0},
};


const PROGMEM trajElem start_yellow[]={
		//Début trajectoire yellow
		{-200,0,13,DISTANCE},
		QUART_TOUR_NEG, //compte pour 3 instructions
		{0,0,0},
};


sState *testTrajyellowInit()
{
	static int i=0; //indice de la pos ds la traj
	static unsigned long prev_millis=0;

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	if(radarIntrusion()>0){
		 return &sPause;
	}
	bool finish=false;

	if (digitalRead(PIN_COLOR)==COLOR_BLUE)
	{
		finish=periodicFunction(start_blue,&st_saveTime,&i,&prev_millis);
		Serial.println("Traj Yellow running ...");
	}
	if (digitalRead(PIN_COLOR)==COLOR_YELLOW)
	{
		finish=periodicFunction(start_yellow,&st_saveTime,&i,&prev_millis);
		Serial.println("Traj Blue running ...");
	}
	if(finish)
	{
		move(0,0);
		return &sRecup;
	}
	if(i>=1){
		uint16_t limits[RAD_NB_PTS]={0,0,0,0};
		radarSetLim(limits);
	}

	return 0;
}

sState sTrajyellowInit={
		BIT(E_MOTOR)|BIT(E_RADAR),
		&initTrajyellowInit,
		&deinitTrajyellowInit,
		&testTrajyellowInit
};
//*****************************************************************************************************************


void initTrajblue(sState *prev)
{
#ifdef DEBUG
	Serial.println(F("debut traj blue"));
#endif

	if (prev==&sPause)
	{
#ifdef DEBUG
		Serial.println(F("\tback from pause"));
#endif
		st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		_backFromPause = 1;
		pause_time+=(millis()-start_pause);
	}
	uint16_t limits[RAD_NB_PTS]={0, 20, 0, 0};
	radarSetLim(limits);
}

void deinitTrajblueInit(sState *next)
{
	if (next==&sPause)
	{
		st_prevSaveTime=st_saveTime;
		st_saveTime=millis();
		st_prevSaveTime_radar=st_saveTime_radar;
		st_saveTime_radar=millis();
	}
	else
	{
		st_saveTime=0;
		st_prevSaveTime=0;
		st_saveTime_radar=0;
		st_prevSaveTime_radar=0;
	}
}



sState *testTrajblue()
{
	static int i=0; //indice de la pos ds la traj
	static unsigned long prev_millis=0;

	uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	if (periodicFunction(start_blue,&st_saveTime,&i,&prev_millis))
	{
		move(0,0);
		return &sRecup;
	}
	/*
		if (radarIntrusion())
		 {
			 start_pause=millis();
			 return &sDead;
		 }*/
	return 0;
}


sState sTrajblueInit={
		BIT(E_MOTOR),//|BIT(E_RADAR),
		&initTrajblue,
		&deinitTrajblueInit,
		&testTrajblue
};


