
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
#include "state_funny_action.h"
#include "state_wait.h"
#include "state_dead.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
periodicTraj periodicFunction = &periodicProgTraj;
#endif

static unsigned long pause_time =0;
static unsigned long start_pause=0;
#define TIME_TO_TRAVEL 75000

void initTrajGreenInit(sState *prev)
	{
		#ifdef DEBUG
			Serial.println(F("debut traj vert (premier trajet)"));
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
		move(0,0);
		}
}

const PROGMEM trajElem start_purple[]={
//Début trajectoire purple
	{0,0,5000},
	{400,15,1000},
	{0,0,0},//Stop
};
const PROGMEM trajElem start_green[]={
	//Début trajectoire green
	{0,0,500},
	{-200,0,1200},
	{0,90,100},
	{-100,90,3600},
	{0,0,100},
	{-300,0,1660},
	{0,0,500},
	{200,0,2000},
	{0,0,0},
};


sState *testTrajGreenInit()
{
	static int i=0; //indice de la pos ds la traj
    static unsigned long prev_millis=0;
    static int flag_end = 0;
    uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	if(!flag_end){
		if(periodicFunction(start_green,&st_saveTime,&i,&prev_millis)){
			#ifdef DEBUG
				Serial.println(F("\tTrajet vert 1 fini !"));
			#endif
			flag_end = 1;
			pause_time=0;
			move(0,0);
		}
	}
	else{

		radarSetLim(limits);
		static unsigned long start_move=millis();

		#ifdef DEBUG
				Serial.println(millis()-start_move-pause_time);
		#endif

		if( (millis()-start_move-pause_time)>TIME_TO_TRAVEL+10000 ){
			move(0,0);
			return &sDead;
		}
		/*else if( (millis()-start_move-pause_time)>TIME_TO_TRAVEL ){
			sTrajGreenInit.flag &= ~BIT(E_RADAR);
			move(-300,0);
		}
		else
		{
			move(-500,0);
			sTrajGreenInit.flag |= BIT(E_RADAR);
		}*/

		if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
			move(0,0);
			return &sDead;
		}
	}
	/*switch(i){

	}*/
	/*if (radarIntrusion())
	 {
		 start_pause=millis();
		 return &sDead;
	 }*/
	return 0;
}

sState sTrajGreenInit={
	BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
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
    //uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	if(!flag_end){
		if(periodicFunction(start_purple,&st_saveTime,&i,&prev_millis)){
			#ifdef DEBUG
				Serial.println(F("\tTrajet 1 fini !"));
			#endif
			flag_end = 1;
			pause_time=0;
			move(0,0);
		}
	}
	else{

		static unsigned long start_move=millis();
#ifdef DEBUG
		Serial.println(millis()-start_move-pause_time-TIME_TO_TRAVEL);
#endif
		if( (millis()-start_move-pause_time)>TIME_TO_TRAVEL+10000 ){
			move(0,0);
			return &sDead;
		}
		/*else if( (millis()-start_move-pause_time)>TIME_TO_TRAVEL ){
			sTrajPurpleInit.flag &= ~BIT(E_RADAR);
			move(-300,0);
		}
		else
		{
			move(-500,0);
			sTrajPurpleInit.flag |= BIT(E_RADAR);
		}*/
		if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
			move(0,0);
			return &sDead;
		}
	}
	/*switch(i){
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
	}*/

	 /*if (radarIntrusion())
	 {
		 start_pause=millis();
		 return &sDead;
	 }*/

	 return 0;
}
sState sTrajPurpleInit={
	BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
	&initTrajPurple,
	&deinitTrajPurpleInit,
	&testTrajPurple
};


