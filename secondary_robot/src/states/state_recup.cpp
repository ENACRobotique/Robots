
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_recup.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_funny_action.h"
#include "state_wait.h"
#include "state_dead.h"
#include "state_travers.h"
#include "DynamixelSerial.h"

static unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
static periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
static periodicTraj periodicFunction = &periodicProgTraj;
#endif

static unsigned long pause_time =0;
static unsigned long start_pause=0;

#ifdef DYN_USE
int delta_dyn=50;
#endif
int unsigned drop_time=2000;
int unsigned up_time=1500;

static int nb_recup= 0;

void initRecup(sState *prev)
{
#ifdef DEBUG
	Serial.println(F("debut Recuperation"));
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

void deinitRecup(sState *next)
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
const PROGMEM trajElem aller_retour[]={
		{-300,0,1500,TEMPS},
		{300,0,15,DISTANCE},
		{0,0,0}
};

sState *testRecup()
{
	static int i=0; //indice de la pos ds la traj
	static int step=0;
	static unsigned long prev_millis=0;
	static int flag_end = 0;
	static int time_for_pompe=0;

#ifdef TIME_FOR_FUNNY_ACTION
	if((millis()-_matchStart) > TIME_FOR_FUNNY_ACTION ) return &sFunnyAction;
#endif

	if(!flag_end){

		analogWrite(PIN_POMPE_PWM,255);
		if(periodicFunction(aller_retour,&st_saveTime,&i,&prev_millis))
		{
			nb_recup++;
			step++;
			move(0,0);
			pause_time=0;
			flag_end=1;
			Dynamixel.ledStatus(NUM_DYNAMIXEL,OFF);
		}
	}
	else
	{
		switch(step)
		{
#ifdef DYN_UP
		case 1:
			Dynamixel.move(NUM_DYNAMIXEL,DYN_UP);
			step++;
			time_for_pompe=millis();
			break;
		case 2:
			//if(abs(abs(Dynamixel.readPosition(NUM_DYNAMIXEL))-DYN_UP)<delta_dyn)
			//if(abs(Dynamixel.readPosition(NUM_DYNAMIXEL)==DYN_UP))
			if(millis()-time_for_pompe>up_time)
			{step++;delay(5);}
			break;
		case 3:
			analogWrite(PIN_POMPE_PWM,0);
			time_for_pompe=millis();
			step++;
			//on part comme ça
			if(nb_recup==4)
				step=6;
			break;
		case 4:
			if(millis()-time_for_pompe>drop_time)
			{
				step++;
				Dynamixel.move(NUM_DYNAMIXEL,DYN_DOWN);
			}
			break;
		case 5:
			if(abs(abs(Dynamixel.readPosition(NUM_DYNAMIXEL))-DYN_DOWN)<delta_dyn)
			{
				step++;
			}
			break;
		case 6:
			Dynamixel.ledStatus(NUM_DYNAMIXEL,ON);
			if(nb_recup<4)
			{
				//reset all static var
				i=0;
				step=0;
				prev_millis=0;
				flag_end = 0;
				time_for_pompe=0;
				return &sRecup;
			}
			else{
				if (digitalRead(PIN_COLOR)==COLOR_BLUE)
					return &sTraverseBlue;
				if (digitalRead(PIN_COLOR)==COLOR_YELLOW)
					return &sTraverseYellow;
			}
			break;
#else
		default:
			analogWrite(PIN_POMPE_PWM,0);
			return &sDead;

#endif
		}
	}
	return 0;
}

sState sRecup={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
		&initRecup,
		&deinitRecup,
		&testRecup
};

