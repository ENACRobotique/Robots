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
#include "tools.h"
#include "params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wall.h"
#include "state_wait.h"
#include "state_dead.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,TimeToLauncher=0;

void initTrajRed(sState *prev)
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

void deinitTrajRed(sState *next)
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
	{0,-15,200}, //to wait until the servo is in the good position
	{-100,-15,3000},
	{-100,20,3000},
	{-100,0,1000},
	{0,0,1000},
	{100,0,1000},
	{0,0,0},
};

sState *testTrajRed()
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

sState sTrajRed={
        BIT(E_MOTOR)/*|BIT(E_RADAR)*/,
        &initTrajRed,
        &deinitTrajRed,
        &testTrajRed
};
//*****************************************************************************************************************


void initTrajYellow(sState *prev)
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

void deinitTrajYellow(sState *next)
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
sState *testTrajYellow()
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
sState sTrajYellow={
		BIT(E_MOTOR) /*|BIT(E_RADAR)*/,
        &initTrajYellow,
        &deinitTrajYellow,
        &testTrajYellow
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
				return &sDead;
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
	    	return &sDead;
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
