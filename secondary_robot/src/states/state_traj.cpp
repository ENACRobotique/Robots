
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
	    uint16_t limits[RAD_NB_PTS]={40};
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
	//Début trajectoire vers cabines de plage
	{-300,-11,1100},
	{-400,0,1200},
	{-300,11,1100},
	{-200,0,1000},
	{0,0,100},//1ere porte fermée
	{300,0,1300},
	{0,-90,400},
	{300,-90,1300},
	{0,0,400},
	{300,0,1700},
	{0,90,400},
	{300,90,1200},
	{0,0,400},
	{-300,0,1000},
	{-200,0,1400},
	{0,0,100},//2eme porte fermée
	{300,0,1700},
	{0,-90,400},
	{300,-90,1400},
	{0,0,300},
	{-300,0,4000},
	{0,0,0},
};

sState *testTrajGreenInit()
	{

	static int i=0;
    static unsigned long prev_millis=0;
    static int flag_end = 0;
    	if(!flag_end){
		    if(periodicFunction(start_green,&st_saveTime,&i,&prev_millis)){
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
				Serial.println("debut traj violet");
			#endif


		    if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("\tback from pause");
				#endif
		        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		        _backFromPause = 1;
		    	}
		    uint16_t limits[RAD_NB_PTS]={40};
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

trajElem start_purple[]={
		//Début trajectoire vers cabines de plage
						{-300,15,1100},
						{-400,0,1500},
						{-300,-15,900},
						{-200,0,900},
						{0,0,100},//1ere porte fermée
						{300,0,1300},
						{0,90,400},
						{300,90,1200},
						{0,0,400},
						{300,0,1700},
						{0,-90,400},
						{300,-90,1300},
						{0,0,400},
						{-300,0,1000},
						{-200,0,1500},
						{0,0,100},//2eme porte fermée
						{300,0,1400},
						{0,90,400},
						{300,90,1300},
						{0,-35,300},
						{-300,-35,2350},
						{0,0,0},
				};

sState *testTrajPurple()
	{
	static int i=0;
    static unsigned long prev_millis=0;
    static int flag_end = 0;
    	if(!flag_end){
		    if(periodicFunction(start_purple,&st_saveTime,&i,&prev_millis)){
				#ifdef DEBUG
					Serial.println("\tTrajet 1 fini !");
				#endif
		    	flag_end = 1;
		    }
    	}
    	else{
    		move(-300,0);
		    if (digitalRead(PIN_SWITCH_LEFT) && digitalRead(PIN_SWITCH_RIGHT)){
				move(0,0);
				return &sRecalage;
			}
    	}

		 if (radarIntrusion()) return &sPause;
	    return 0;
	}
sState sTrajPurpleInit={
		BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initTrajPurple,
        &deinitTrajYellowInit,
        &testTrajPurple
};





