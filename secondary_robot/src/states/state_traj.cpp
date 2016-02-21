
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_line.h"
#include "lib_wall.h"
#include "lib_attitude.h"
#include "lib_heading.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_Recalage.h"
#include "state_wait.h"
#include "lib_radar_mask.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,st_saveTime_radar=0,st_prevSaveTime_radar=0;
#ifdef HEADING
periodicTraj periodicFunction = &periodicProgTrajHeading;
#else
periodicTraj periodicFunction = &periodicProgTraj;
#endif

void initTrajGreenInit(sState *prev)
	{
		#ifdef DEBUG
			Serial.println("debut traj rouge (premier trajet)");
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

trajElem start_green[]={
#ifdef HEADING
				{30,0,5500},
				{0,-90,9000},
				{0,-90,2000},
				{10,-90,3000},
				{0,0,0},
#else
				//Début trajectoire vers cabines de plage
				{0,60,300},
				{250,60,1600},
				{0,0,100},
				{-300,0,4000},
				{0,0,0},
#endif
				};

radarElem start_green_radar[]={
				{40,25,5500},
				{0,0,9000},
				{0,0,2000},
				{30,30,3000},
				{0,0,0},
				};

sState *testTrajGreenInit()
	{
		static int i=0;
		static int i_radar=0;
	    static unsigned long prev_millis=0;
	    static unsigned long prev_millis_radar=0;

	    if(periodicFunction(start_green,&st_saveTime,&i,&prev_millis))
	    {

			#ifdef DEBUG
				Serial.println("\tTrajet 1 fini !");
			#endif

	    	 return &sRecalage;
	    }
	    if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT))
	    	{
	    		move(0,0);
	    		return &sRecalage;
	    	}
	    if(periodicProgRadarLimit(start_green_radar,&st_saveTime_radar,&i_radar,&prev_millis_radar)){
			#ifdef DEBUG
				Serial.println("\tFin radar 1 !");
			#endif
		}

	    if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajGreenInit={
        BIT(E_MOTOR)/*|BIT(E_RADAR)|BIT(E_HEADING)*/,
        &initTrajGreenInit,
        &deinitTrajGreenInit,
        &testTrajGreenInit
};
//*****************************************************************************************************************


void initTrajPurple(sState *prev)
	{
		    #ifdef DEBUG
				Serial.println("debut traj jaune");
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

void deinitTrajYellowInit(sState *next)
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

trajElem start_yellow[]={
#ifdef HEADING
		{30,0,5500},
		{0,90,9000},
		{0,90,2000},
		{10,90,2000},
		{0,0,0},
#else
		//Début trajectoire vers cabines de plage

						{-300,15,1100},
						{-400,0,1200},
						{-300,-15,1100},
						{-200,0,1000},
						{0,0,100},//1ere porte fermée
						{300,0,1300},
						{0,90,400},
						{300,90,1200},
						{0,0,400},
						{300,0,1900},
						{0,-90,400},
						{300,-90,1300},
						{0,0,400},
						{-300,0,1000},
						{-200,0,1400},
						{0,0,100},//2eme porte fermée
						{300,0,1300},
						{0,90,400},
						{300,90,1200},
						{0,0,300},
						{-300,0,4000},
#endif
						{0,0,0},
				};

radarElem start_yellow_radar[]={
				{40,25,5500},
				{0,0,9000},
				{0,0,2000},
				{30,30,3000},
				{0,0,0},
				};
sState *testTrajPurple()
	{
	static int i=0;
    static unsigned long prev_millis=0;
    static int flag_end = 0;
    	if(!flag_end){
		    if(periodicFunction(start_yellow,&st_saveTime,&i,&prev_millis)){
				#ifdef DEBUG
					Serial.println("\tTrajet 1 fini !");
				#endif
		    	flag_end = 1;
		    }
    	}
    	else{
    		move(-200,0);
		    if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
				move(0,0);
				return &sRecalage;
			}
    	}

		 if (radarIntrusion()) return &sPause;
	    return 0;
	}
sState sTrajYellowInit={
		BIT(E_MOTOR)/*|BIT(E_RADAR)|BIT(E_HEADING)*/,
        &initTrajPurple,
        &deinitTrajYellowInit,
        &testTrajPurple
};





