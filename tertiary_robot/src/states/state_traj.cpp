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
#include "state_clap.h"
#include "state_wall.h"
#include "state_wait.h"
#include "state_dead.h"

static unsigned long st_saveTime=0,st_prevSaveTime=0,TimeToLauncher=0;

void initTrajGreen(sState *prev)
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
	    uint16_t limits[RAD_NB_PTS]={40,40};
	   	    		radarSetLim(limits);
	}

void deinitTrajGreen(sState *next)
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

trajElem start_green[]={
	{-500,0,7500},
	{-200,45,7500},
	{500,-3,8000},
	{0,0,0}
};

sState *testTrajGreen()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(start_green,&st_saveTime,&i,&prev_millis))
	    {
	    	 return &sClap1_GREEN;
	    }

		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }

	    if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajGreen={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajGreen,
        &deinitTrajGreen,
        &testTrajGreen
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
		    uint16_t limits[RAD_NB_PTS]={40,40};
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
		{-400,0,5000},
		{-200,-45,1000},
		{400,0,1000},
		{0,0,0}
};
sState *testTrajYellow()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
		    if(periodicProgTraj(start_yellow,&st_saveTime,&i,&prev_millis))
		   	    {
		    	return &sClap1_YELLOW;
		   	    }
		 if (radarIntrusion()) return &sPause;
		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }
	    return 0;
	}
sState sTrajYellow={
		BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajYellow,
        &deinitTrajYellow,
        &testTrajYellow
};


//******************************************************************************************************************
void initTrajYellowFinal(sState *prev)
	{

			#ifdef DEBUG
				Serial.println("debut traj jaune final");
			#endif
		    if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("\tback from pause");
				#endif
		        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		    	}
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

trajElem yellowFinal[]={
		        {50,90,1000},
		};

sState *testTrajYellowFinal()
	{
	static int i=0;
		    static unsigned long prev_millis=0;

			if(periodicProgTraj(yellowFinal,&st_saveTime,&i,&prev_millis))
			 	{
				return &sDead;
			 	}


			 if (millis() - _matchStart >= TIME_MATCH_STOP){
				 emergencyStop();
				 return &sDead;
			 }

		if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajYellowFinal={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajYellowFinal,
        &deinitTrajYellowFinal,
        &testTrajYellowFinal
};


//******************************************************************************************************************
void initTrajGreenFinal(sState *prev)
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

void deinitTrajGreenFinal(sState *next)
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

trajElem GreenFinal[]={
        {50,90,500},
       };

sState *testTrajGreenFinal()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(GreenFinal,&st_saveTime,&i,&prev_millis))
		{
	    	return &sDead;
		}


		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }


	if (radarIntrusion()) return &sPause;
	 	    return 0;
	}

sState sTrajGreenFinal={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajGreenFinal,
        &deinitTrajGreenFinal,
        &testTrajGreenFinal
};

//********************************************************************************
void initTrajBetweenClaps_YELLOW(sState *prev)
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

void deinitTrajBetweenClaps_YELLOW(sState *next)
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

trajElem BetweenClaps_YELLOW[]={
        {50,-5,500},
       };

sState *testTrajBetweenClaps_YELLOW()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(BetweenClaps_YELLOW,&st_saveTime,&i,&prev_millis))
		{
	    	return &sClap2;
		}


		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }


	if (radarIntrusion()) return &sPause;
	 	    return 0;
	}

sState sTrajBetweenClaps_YELLOW={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajBetweenClaps_YELLOW,
        &deinitTrajBetweenClaps_YELLOW,
        &testTrajBetweenClaps_YELLOW
};


void initTrajBetweenClaps_GREEN(sState *prev)
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

void deinitTrajBetweenClaps_GREEN(sState *next)
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

trajElem BetweenClaps_GREEN[]={
        {50,5,500},
       };

sState *testTrajBetweenClaps_GREEN()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(BetweenClaps_YELLOW,&st_saveTime,&i,&prev_millis))
		{
	    	return &sClap2;
		}


		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }


	if (radarIntrusion()) return &sPause;
	 	    return 0;
	}

sState sTrajBetweenClaps_GREEN={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajBetweenClaps_GREEN,
        &deinitTrajBetweenClaps_GREEN,
        &testTrajBetweenClaps_GREEN
};
