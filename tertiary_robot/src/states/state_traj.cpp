
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wait.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
periodicTraj periodicFunction = &periodicProgTraj;
#endif

void initTrajGreenStart(sState *prev)
{
	#ifdef DEBUG
		Serial.println("debut traj Vert");
	#endif

	if (prev==&sPause)
	{
		#ifdef DEBUG
			Serial.println("\tback from pause");
		#endif
		st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		st_saveTime_radar=millis()-st_saveTime_radar+st_prevSaveTime_radar;
		_backFromPause = 1;
	}
	uint16_t limits[RAD_NB_PTS]={40,40};
				radarSetLim(limits);
}

void deinitTrajGreenStart(sState *next)
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

trajElem start_green[]={
	{30,3,1500},
	{30,-3,1500},
	{-20,-10,4000},
	{0,0,0},
};


sState *testTrajGreenStart()
{
	static int i=0;
	static unsigned long prev_millis=0;

	if(periodicFunction(start_green,&st_saveTime,&i,&prev_millis))
	{

		#ifdef DEBUG
			Serial.println("\tTrajet 1 fini !");
		#endif

		 return &sWait;
	}

	if (radarIntrusion()) return &sPause;
	return 0;
}

sState sTrajGreenStart={
	BIT(E_MOTOR)|BIT(E_RADAR)|BIT(E_HEADING),
	&initTrajGreenStart,
	&deinitTrajGreenStart,
	&testTrajGreenStart
};
//*****************************************************************************************************************


void initTrajPurpleInit(sState *prev)
{
	#ifdef DEBUG
		Serial.println("debut traj Violet");
	#endif
	if (prev==&sPause)
	{
		#ifdef DEBUG
			Serial.println("\tback from pause");
		#endif
		st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		_backFromPause = 1;
	}
	uint16_t limits[RAD_NB_PTS]={40,40};
	radarSetLim(limits);
}

void deinitTrajPurpleInit(sState *next)
{
	if (next==&sPause)
	{
		st_prevSaveTime=st_saveTime;
		st_saveTime=millis();
	}
	else
	{
		#ifdef DEBUG
			Serial.println("Fin traj jaune");
		#endif
		st_saveTime=0;
		st_prevSaveTime=0;
	}
}

trajElem start_purple[]={
	{30,0,4500},
	{20,10,6000},
	{15,0,750},
	{0,0,0},
};

sState *testTrajPurpleInit()
{
	static int i=0;
	static unsigned long prev_millis=0;
	if(periodicFunction(start_purple,&st_saveTime,&i,&prev_millis))
	{
		#ifdef DEBUG
			Serial.println("\tTrajet 1 fini !");
		#endif
		return 0;
	}

	 if (radarIntrusion()) return &sPause;
	return 0;
}

sState sTrajPurpleStart={
		BIT(E_MOTOR)|BIT(E_RADAR)|BIT(E_HEADING),
        &initTrajPurpleInit,
        &deinitTrajPurpleInit,
        &testTrajPurpleInit
};
