
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_Recalage.h"
#include "state_wait.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
periodicTraj periodicFunction = &periodicProgTraj;
#endif

static unsigned long pause_time =0;
static unsigned long start_pause=0;
#define TIME_TO_TRAVEL 5750

void initTrajGreenInit(sState *prev)
	{
		#ifdef DEBUG
			Serial.println(F("debut traj rouge (premier trajet)"));
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
	    uint16_t limits[RAD_NB_PTS]={25,0,0,0};
	   	radarSetLim(limits);
		#ifdef DEBUG
	   		Serial.println(pause_time);
		#endif

	}

void deinitTrajGreenInit(sState *next)
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

const PROGMEM trajElem start_purple[]={
//Début trajectoire vers cabines de plage
	{0,-18,100},//Radar active 0
	{-300,18,1100},
	{-400,0,1500},
	{-300,-20,900},//Radar inactive 3
	{-200,0,1200},
	{0,0,100},//1ere porte fermée
	{300,0,1300},
	{0,90,400},
	{300,90,1200},
	{0,0,400},//Radar active 9
	{300,0,1800},
	{0,-90,400},//Radar inactive 11
	{300,-90,1300},
	{0,0,400},
	{-300,0,1000},
	{-200,0,1600},
	{0,0,100},//2eme porte fermée
	{300,0,1400},
	{0,90,400},
	{300,90,1300},
	{0,-35,300},// Radar active 20
	{-300,-35,2300},
	{0,0,0},
};
const PROGMEM trajElem start_green[]={
	//Début trajectoire vers cabines de plage
	//{0,0,3600000},
	{0,-15,100},//Radar active 0
	{-300,-15,1100},
	{-400,0,1500},
	{-300,25,800},
	{-200,0,1450},//Radar inactive 3
	{0,0,100},//1ere porte fermée
	{300,0,1300},
	{0,-90,400},
	{300,-90,1400},
	{0,0,400},//Radar active 9
	{300,0,1700},
	{0,90,400},//Radar inactive 11
	{300,90,1300},
	{0,0,400},
	{-300,0,1000},
	{-200,0,1650},
	{0,0,100},//2eme porte fermée
	{300,0,1400},
	{0,-90,400},
	{300,-90,1300},//Radar active 20
	{0,35,300},
	{-300,35,2150},
	{0,0,0},
};


sState *testTrajGreenInit()
{
	static int i=0;
    static unsigned long prev_millis=0;
    static int flag_end = 0;
    uint16_t limits[RAD_NB_PTS]={30, 30,0, 0};

	if(!flag_end){
		if(periodicFunction(start_green,&st_saveTime,&i,&prev_millis)){
			#ifdef DEBUG
				Serial.println(F("\tTrajet 1 fini !"));
			#endif
			flag_end = 1;
			pause_time=0;
		}
	}
	else{

		radarSetLim(limits);
		static unsigned long start_move=millis();
		if( (millis()-start_move-pause_time)>TIME_TO_TRAVEL )
		{
			sTrajGreenInit.flag &= ~BIT(E_RADAR);
			move(-300,0);
		}
		else
		{
			move(-500,0);
			sTrajGreenInit.flag |= BIT(E_RADAR);
		}

		if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
			move(0,0);
			return &sRecalage;
		}
	}
	switch(i){
		case 3:
			sTrajGreenInit.flag &= ~BIT(E_RADAR);
			break;
		case 9:
			sTrajGreenInit.flag |= BIT(E_RADAR);
			break;
		case 11:
			sTrajGreenInit.flag &= ~BIT(E_RADAR);
			break;
		case 20:
			sTrajGreenInit.flag |= BIT(E_RADAR);
			break;
	}
	 if (radarIntrusion())
	 {
		 start_pause=millis();
		 return &sPause;
	 }
	return 0;
}

sState sTrajGreenInit={
	BIT(E_MOTOR)|BIT(E_RADAR),
	&initTrajGreenInit,
	&deinitTrajGreenInit,
	&testTrajGreenInit
};
//*****************************************************************************************************************


void initTrajPurple(sState *prev)
{
	#ifdef DEBUG
		Serial.println(F("debut traj violet"));
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

void deinitTrajPurpleInit(sState *next)
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



sState *testTrajPurple()
{
	static int i=0;
    static unsigned long prev_millis=0;
    static int flag_end = 0;
    uint16_t limits[RAD_NB_PTS]={30, 30,0, 0};
	if(!flag_end){
		if(periodicFunction(start_purple,&st_saveTime,&i,&prev_millis)){
			#ifdef DEBUG
				Serial.println(F("\tTrajet 1 fini !"));
			#endif
			flag_end = 1;
			pause_time=0;
		}
	}
	else{

		static unsigned long start_move=millis();
		if( (millis()-start_move-pause_time)>TIME_TO_TRAVEL ){
			sTrajPurpleInit.flag &= ~BIT(E_RADAR);
			move(-300,0);
		}
		else
		{
			move(-500,0);
			sTrajPurpleInit.flag |= BIT(E_RADAR);
		}
		if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
			move(0,0);
			return &sRecalage;
		}
	}
	switch(i){
	case 3:
		sTrajPurpleInit.flag &= ~BIT(E_RADAR);
		break;
	case 9:
		radarSetLim(limits);
		sTrajPurpleInit.flag |= BIT(E_RADAR);
		break;
	case 11:
		sTrajPurpleInit.flag &= ~BIT(E_RADAR);
		break;
	case 20:
		sTrajPurpleInit.flag |= BIT(E_RADAR);
		break;
	}
	 if (radarIntrusion())
	 {
		 start_pause=millis();
		 return &sPause;
	 }
	 return 0;
}
sState sTrajPurpleInit={
	BIT(E_MOTOR)|BIT(E_RADAR),
	&initTrajPurple,
	&deinitTrajPurpleInit,
	&testTrajPurple
};


