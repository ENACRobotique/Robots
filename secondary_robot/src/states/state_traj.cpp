
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_funny_action.h"
#include "state_wait.h"
#include "state_dead.h"
#include "DynamixelSerial.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
periodicTraj periodicFunction = &periodicProgTraj;
#endif

static unsigned long pause_time =0;
static unsigned long start_pause=0;
#define TIME_TO_TRAVEL 75000

void initTrajyellowInit(sState *prev)
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
	uint16_t limits[RAD_NB_PTS]={25,0,0,0};
	radarSetLim(limits);
#ifdef DEBUG
	Serial.println(pause_time);
#endif

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
	}
}

const PROGMEM trajElem start_blue[]={
		//Début trajectoire blue
		/*
	{0,90,500,TEMPS},
	{250,90,71.5,DISTANCE},*/
		QUART_TOUR_POS,
		{0,0,1000,TEMPS},
		{0,0,0},//Stop
};
const PROGMEM trajElem aller_retour[]={
		{-300,0,1500,TEMPS},
		{300,0,15,DISTANCE},
		{0,0,0},
};
#define Recup {-300,0,1500,TEMPS},{300,0,15,DISTANCE},{0,0,1500,TEMPS},{0,0,2000,TEMPS},{0,0,1000,TEMPS}
//				Pompe on 	4 		Dyn up		6						Pompe off	7		Dyn down	8

const PROGMEM trajElem start_yellow[]={
		//Début trajectoire yellow
		{200,0,-13,DISTANCE},
		QUART_TOUR_NEG, //compte pour 3 instructions
		{0,0,0},
};


sState *testTrajyellowInit()
{
	static int i=0; //indice de la pos ds la traj
	static int nb_recup= -1;
	static int step=0;
	static unsigned long prev_millis=0;
	static int flag_end = 0;
	static int time_for_pompe=0;

	uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	if(!flag_end){
		switch(step)
		{
		case 0:
			if(nb_recup<0)
			{
				if(periodicFunction(start_yellow,&st_saveTime,&i,&prev_millis))
				{
					nb_recup++;
					move(0,0);
					pause_time=0;
				}
			}
			else
			{
				analogWrite(PIN_POMPE_PWM,255);
				if(periodicFunction(aller_retour,&st_saveTime,&i,&prev_millis))
				{
					nb_recup++;
					step++;
					move(0,0);
					pause_time=0;
				}
			}
			break;
		case 1:
			Dynamixel.move(NUM_DYNAMIXEL,DYN_UP);

			//if(abs(Dynamixel.readPosition(NUM_DYNAMIXEL)-DYN_UP)<10)
			if(abs(Dynamixel.readPosition(NUM_DYNAMIXEL)==DYN_UP))
			{step++;}
			break;
		case 2:
			analogWrite(PIN_POMPE_PWM,0);
			time_for_pompe=millis();
			step++;
			//on part comme ça
			if(nb_recup==4)
				flag_end=true;
			break;
		case 3:
			if(millis()-time_for_pompe>2000)
			{
				step++;
				Dynamixel.move(NUM_DYNAMIXEL,DYN_DOWN);
			}
			break;
		case 4:
			if(abs(Dynamixel.readPosition(NUM_DYNAMIXEL)-DYN_DOWN)<10)
				step=0;
			break;

		}
	}
	else{
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

sState sTrajyellowInit={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
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
	static int i=0;
	static unsigned long prev_millis=0;

	static int flag_end = 0;
	//uint16_t limits[RAD_NB_PTS]={0,0,0, 0};

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	if(!flag_end){
		if(periodicFunction(start_blue,&st_saveTime,&i,&prev_millis)){
#ifdef DEBUG
			Serial.println(F("\tTrajet blue fini !"));
#endif
			flag_end = 1;
			pause_time=0;
			move(0,0);
#ifdef DYN_USE
			Dynamixel.move(NUM_DYNAMIXEL,80);
			delay(1000);
			Dynamixel.move(NUM_DYNAMIXEL,800);
#endif
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

	}



	return 0;
}
sState sTrajblueInit={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
		&initTrajblue,
		&deinitTrajblueInit,
		&testTrajblue
};


