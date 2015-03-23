/*
 * state_traj.cpp
 *
 *  Created on: 8 mai 2013
 *      Author: quentin
 *   Modify on: janvier 2014
 *   		By: Seb
 */


#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_line.h"
#include "lib_wall.h"
#include "lib_trajectory.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wall.h"
#include "state_funny.h"
#include "state_wait.h"
#include "state_fresco.h"
#include "state_lineMonit.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,TimeToLauncher=0;
//int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);
void initTrajRedInit(sState *prev)
	{
		#ifdef DEBUG
			Serial.println("debut traj rouge");
		#endif

	    if (prev==&sPause)
	    	{
			#ifdef DEBUG
				Serial.println("\tback from pause");
			#endif
	        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
	    	}
	    uint16_t limits[RAD_NB_PTS]={3,40};
	   	    		radarSetLim(limits);
	}

void deinitTrajRedInit(sState *next)
	{
	    if (next==&sPause)
	    	{
	        st_prevSaveTime=st_saveTime;
	        st_saveTime=millis();
	    	}
	    else
	    	{
	        st_saveTime=0;
	        st_prevSaveTime=0;
	    	}
}

trajElem start_red[]={
		{0,0,100}, //to wait until the servo is in the good position
				{100,0,1000},
				{0,10,1000},
				{200,20,1000},
				{0,30,1000},
				{300,40,1000},
				{0,0,1000},
				{400,50,1000},
				{0,0,1000},
				{500,100,1000},
				{0,0,1000},
				{600,120,1000},
				{0,0,1000},
				{-300,150,1000},
				{0,180,1000},
				{-600,10,1000},
				{0,0,1000},
				{360,90,10000},
				{0,0,1000},
				{0,0,0},
       };

sState *testTrajRedInit()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(start_red,&st_saveTime,&i,&prev_millis))
	    {

	    	launcherServoDown.write(120);
	    	launcherServoUp.write(10);

	    	 return &sTrajRedFinal;
	    }

	    if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajRedInit={
        BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initTrajRedInit,
        &deinitTrajRedInit,
        &testTrajRedInit
};
//*****************************************************************************************************************


void initTrajYellowInit(sState *prev)
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
		    	}
		    uint16_t limits[RAD_NB_PTS]={40,3};
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
		        st_saveTime=0;
		        st_prevSaveTime=0;
		    	}
}

trajElem start_yellow[]={ //A MODIFIER
	    {0,0,0},
		};
sState *testTrajYellowInit()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
		    if(periodicProgTraj(start_yellow,&st_saveTime,&i,&prev_millis))
		   	    {
		    	launcherServoDown.write(120);
		    	launcherServoUp.write(10);
		    	return &sTrajYellowFinal;
		   	    }
		 if (radarIntrusion()) return &sPause;
	    return 0;
	}
sState sTrajYellowInit={
		BIT(E_MOTOR) /*|BIT(E_RADAR)*/,
        &initTrajYellowInit,
        &deinitTrajYellowInit,
        &testTrajYellowInit
};


//******************************************************************************************************************
void initTrajYellowFinal(sState *prev)
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
		    	}
		    	else{TimeToLauncher = millis() ;}
		    uint16_t limits[RAD_NB_PTS]={30,3};
		    	radarSetLim(limits);
	}

void deinitTrajYellowFinal(sState *next)
	{
		    if (next==&sPause)
		    	{
		        st_prevSaveTime=st_saveTime;
		        st_saveTime=millis();
		    	}
		    else
		    	{
		        st_saveTime=0;
		        st_prevSaveTime=0;
		    	}
}

trajElem Final_yellow[]={
		        {0,0,0},
		};

sState *testTrajYellowFinal()
	{
	static int i=0;
		    static unsigned long prev_millis=0;

			if(periodicProgTraj(Final_yellow,&st_saveTime,&i,&prev_millis))
			 	{
				return &sFunny;
			 	}


			if((millis()-st_saveTime)-TimeToLauncher > 1500 && TimeToLauncher!=0 ){
				    	 	 launcherServoDown.write(15);
				    	 	TimeToLauncher=0;
				    	 	 }


			if((millis()-st_saveTime)-TimeToLauncher > 3000 ){
				    	 uint16_t limits[RAD_NB_PTS]={3,40};
				    		    		radarSetLim(limits);
				   	     	}

		if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajYellowFinal={
        BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initTrajYellowFinal,
        &deinitTrajYellowFinal,
        &testTrajYellowFinal
};


//******************************************************************************************************************
void initTrajRedFinal(sState *prev)
	{
	  	#ifdef DEBUG
			Serial.println("debut traj rouge");
		#endif

	    if (prev==&sPause)
	    	{
			#ifdef DEBUG
				Serial.println("\tback from pause");
			#endif
	        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
	    	}
	    else{TimeToLauncher = millis() ;}

	    uint16_t limits[RAD_NB_PTS]={3,40};
	    		radarSetLim(limits);
	}

void deinitTrajRedFinal(sState *next)
	{
	    if (next==&sPause)
	    	{
	        st_prevSaveTime=st_saveTime;
	        st_saveTime=millis();
	    	}
	    else
	    	{
	        st_saveTime=0;
	        st_prevSaveTime=0;
	    	}
}

trajElem Final_red[]={
        {0,0,0},
       };

sState *testTrajRedFinal()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(Final_red,&st_saveTime,&i,&prev_millis))
		{
	    	return &sFunny;
		}


	    if((millis()-st_saveTime)-TimeToLauncher > 1500 && TimeToLauncher!=0 ){
	    	launcherServoDown.write(15);
	    	TimeToLauncher=0;
		}


	     if((millis()-st_saveTime)-TimeToLauncher > 3100 ){
	    	 uint16_t limits[RAD_NB_PTS]={40,3};
	    		    		radarSetLim(limits);
	     	     				    	 	 }

	if (radarIntrusion()) return &sPause;
	 	    return 0;
	}

sState sTrajRedFinal={
        BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initTrajRedFinal,
        &deinitTrajRedFinal,
        &testTrajRedFinal
};
