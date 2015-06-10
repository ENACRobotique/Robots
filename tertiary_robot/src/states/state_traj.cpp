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
#include "sharp_2d120x.h"

#define TIME_SHARP 200

static unsigned long st_saveTime=0,st_prevSaveTime=0;

void initTrajStart_GREEN(sState *prev)
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
	        _backFromPause=1;
	    	}
	    int limits[RAD_NB_PTS]={0,15};
	    sharpSetLim(limits);
	}

void deinitTrajStart_GREEN(sState *next)
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

trajElem TrajStart_GREEN[]={
		{-700,0,4400},
		{0,0,0}
};

sState *testTrajStart_GREEN()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(TrajStart_GREEN,&st_saveTime,&i,&prev_millis))
	    {
	    	 return &sTrajToClaps_GREEN;
	    }

		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }

	    return 0;
	}

sState sTrajStart_GREEN={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajStart_GREEN,
        &deinitTrajStart_GREEN,
        &testTrajStart_GREEN
};
//*****************************************************************************************************************


void initTrajStart_YELLOW(sState *prev)
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
		        _backFromPause=1;
		    	}
		    int limits[RAD_NB_PTS]={0,15};
		    	    sharpSetLim(limits);
	}

void deinitTrajStart_YELLOW(sState *next)
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

trajElem TrajStart_YELLOW[]={ //A MODIFIER
		{-700,0,4800},
		{0,0,0}
};
sState *testTrajStart_YELLOW()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
		    if(periodicProgTraj(TrajStart_YELLOW,&st_saveTime,&i,&prev_millis))
		   	    {
		    	return &sTrajToClaps_YELLOW;
		   	    }
		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }
	    return 0;
	}
sState sTrajStart_YELLOW={
		BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajStart_YELLOW,
        &deinitTrajStart_YELLOW,
        &testTrajStart_YELLOW
};


//******************************************************************************************************************
void initTrajToClaps_YELLOW(sState *prev)
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
		        _backFromPause=1;
		    	}
		    int limits[RAD_NB_PTS]={0,15};
		    sharpSetLim(limits);

	}

void deinitTrajToClaps_YELLOW(sState *next)
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

trajElem TrajToClaps_YELLOW[]={
		{-400,40,3300},
		{-500,-7,700},
		{500,-7,6200},
		{0,0,0}
};

sState *testTrajToClaps_YELLOW()
	{
	static int i=0;
		    static unsigned long prev_millis=0;

			if(periodicProgTraj(TrajToClaps_YELLOW,&st_saveTime,&i,&prev_millis))
			 	{
				return &sClap1_YELLOW;
			 	}


			 if (millis() - _matchStart >= TIME_MATCH_STOP){
				 emergencyStop();
				 return &sDead;
			 }

	    return 0;
	}

sState sTrajToClaps_YELLOW={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajToClaps_YELLOW,
        &deinitTrajToClaps_YELLOW,
        &testTrajToClaps_YELLOW
};


//******************************************************************************************************************
void initTrajToClaps_GREEN(sState *prev)
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
	        _backFromPause=1;
	    	}
	    int limits[RAD_NB_PTS]={0,15};
	    sharpSetLim(limits);
	}

void deinitTrajToClaps_GREEN(sState *next)
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

trajElem TrajToClaps_GREEN[]={
		{-400,-40,4100},
		{-500,7,700},
		{500,7,6200},
		{0,0,0}
       };

sState *testTrajToClaps_GREEN()
	{
		static int i=0;
	    static unsigned long prev_millis=0;
	    if(periodicProgTraj(TrajToClaps_GREEN,&st_saveTime,&i,&prev_millis))
		{
	    	return &sClap1_GREEN;
		}


		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }
	 	    return 0;
	}

sState sTrajToClaps_GREEN={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajToClaps_GREEN,
        &deinitTrajToClaps_GREEN,
        &testTrajToClaps_GREEN
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
	        _backFromPause=1;
	    	}
	    int limits[RAD_NB_PTS]={0,15};
	    	    sharpSetLim(limits);
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

trajElem TrajBetweenClaps_YELLOW[]={
        {-400,-7,3000},
        {0,0,0}
       };

sState *testTrajBetweenClaps_YELLOW()
	{
		static int i=0;
	    static unsigned long prev_millis=0;
	    if(periodicProgTraj(TrajBetweenClaps_YELLOW,&st_saveTime,&i,&prev_millis))
		{
	    	return &sClap2_YELLOW;
		}


		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }
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
	        _backFromPause=1;
	    	}
	    int limits[RAD_NB_PTS]={0,15};
	    	    sharpSetLim(limits);
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

trajElem TrajBetweenClaps_GREEN[]={
        {-400,7,3000},
        {0,0,0}
       };

sState *testTrajBetweenClaps_GREEN()
	{
		static int i=0;
	    static unsigned long prev_millis=0;
	    if(periodicProgTraj(TrajBetweenClaps_YELLOW,&st_saveTime,&i,&prev_millis))
		{
	    	return &sClap2_GREEN;
		}


		 if (millis() - _matchStart >= TIME_MATCH_STOP){
			 emergencyStop();
			 return &sDead;
		 }

	 	    return 0;
	}

sState sTrajBetweenClaps_GREEN={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajBetweenClaps_GREEN,
        &deinitTrajBetweenClaps_GREEN,
        &testTrajBetweenClaps_GREEN
};
