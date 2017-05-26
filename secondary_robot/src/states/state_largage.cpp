
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

void initlargyellow(sState *prev)
{
#ifdef DEBUG
	Serial.println(F("debut traj yellow (premier trajet)"));
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
	uint16_t limits[RAD_NB_PTS]={0,0,0,0};
	radarSetLim(limits);
#ifdef DEBUG
	Serial.println(pause_time);
#endif

}

void deinitlargyellow(sState *next)
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
	}
}

const PROGMEM trajElem start_blue[]={
		//Début trajectoire blue
		{0,0,1000,TEMPS},
		{300,1,45,DISTANCE},
		{0,0,0},
};


const PROGMEM trajElem start_yellow[]={
		//Début trajectoire yellow
		{0,0,1000,TEMPS},
		{-350,-2,42,DISTANCE},
		{0,0,0},
};


sState *testlargyellow()
{
	static int i=0; //indice de la pos ds la traj
	static unsigned long prev_millis=0;

	uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	Hodor.write(HODOR_OPEN);
	if (periodicFunction(start_yellow,&st_saveTime,&i,&prev_millis))
	{
		move(0,0);
		return &sDead;
	}
	return 0;
}

sState sLargageYellow={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
		&initlargyellow,
		&deinitlargyellow,
		&testlargyellow
};
//*****************************************************************************************************************


void initlargblue(sState *prev)
{
#ifdef DEBUG
	Serial.println(F("debut largage blue"));
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
	uint16_t limits[RAD_NB_PTS]={0, 0, 0, 0};
	radarSetLim(limits);
}

void deinitlargblue(sState *next)
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



sState *testlargblue()
{
	static int i=0; //indice de la pos ds la traj
	static unsigned long prev_millis=0;

	uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif
	Hodor.write(HODOR_OPEN);
	if (periodicFunction(start_blue,&st_saveTime,&i,&prev_millis))
	{
		move(0,0);
		return &sDead;
	}
	/*
		if (radarIntrusion())
		 {
			 start_pause=millis();
			 return &sDead;
		 }*/
	return 0;
}


sState sLargageBlue={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
		&initlargblue,
		&deinitlargblue,
		&testlargblue
};


