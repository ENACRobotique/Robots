/*
 * state_clap.cpp
 *
 *  Created on: 14 mai 2015
 *      Author: florian
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
#include "state_clap.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wall.h"
#include "state_wait.h"
#include "state_dead.h"
#include "sharp_2d120x.h"

#define TIME_SHARP 200

static unsigned long st_saveTime=0,st_prevSaveTime=0;
Servo servoClap;

void initClap1_YELLOW(sState *prev)
	{

			#ifdef DEBUG
				Serial.println("debut clap1 YELLOW");
			#endif
		    if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("\tback from pause");
				#endif
		        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		        _backFromPause=1;
		    	}
		    else{
		    	if (digitalRead(PIN_COLOR)==COLOR_YELLOW){
		    		servoClap.write(CLAPYELLOW);
		    	}
		    	else{servoClap.write(CLAPGREEN);}
		    }
		    int limits[NB_SHARP]={0,15};
		    	    sharpSetLim(limits);
	}

void deinitClap1_YELLOW(sState *next)
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
		        servoClap.write(CLAPNEUTRAL);
		    	}
}

trajElem Clap1_YELLOW_traj[]={
	{-500,-7,3000},
	{0,0,0}
};

sState *testClap1_YELLOW()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
			if(periodicProgTraj(Clap1_YELLOW_traj,&st_saveTime,&i,&prev_millis))
			 	{
				return &sTrajBetweenClaps_YELLOW;
			 	}

			 if (millis() - _matchStart >= TIME_MATCH_STOP){
				 emergencyStop();
				 return &sDead;
			 }

	    return 0;
	}

sState sClap1_YELLOW={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initClap1_YELLOW,
        &deinitClap1_YELLOW,
        &testClap1_YELLOW
};

void initClap1_GREEN(sState *prev)
	{

			#ifdef DEBUG
				Serial.println("debut clap1 GREEN");
			#endif
		    if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("\tback from pause");
				#endif
		        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		        _backFromPause=1;
		    	}
		    else{
		    	if (digitalRead(PIN_COLOR)==COLOR_YELLOW){
		    		servoClap.write(CLAPYELLOW);
		    	}
		    	else{servoClap.write(CLAPGREEN);}
		    }
		    int limits[RAD_NB_PTS]={0,15};
		    	    sharpSetLim(limits);
	}

void deinitClap1_GREEN(sState *next)
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
		        servoClap.write(CLAPNEUTRAL);
		    	}
}

trajElem Clap1_GREEN_traj[]={
	{-500,7,3000},
	{0,0,0}
};

sState *testClap1_GREEN()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
			if(periodicProgTraj(Clap1_YELLOW_traj,&st_saveTime,&i,&prev_millis))
			 	{
				emergencyStop();
				return &sDead;
			 	}

			 if (millis() - _matchStart >= TIME_MATCH_STOP){
				 return &sTrajBetweenClaps_GREEN;
			 }

	    return 0;
	}

sState sClap1_GREEN={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initClap1_GREEN,
        &deinitClap1_GREEN,
        &testClap1_GREEN
};


void initClap2_YELLOW(sState *prev)
	{

			#ifdef DEBUG
				Serial.println("debut clap2 YELLOW");
			#endif
		    if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("\tback from pause");
				#endif
		        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		        _backFromPause=1;
		    	}
		    else{
		    	if (digitalRead(PIN_COLOR)==COLOR_YELLOW){
		    		servoClap.write(CLAPYELLOW);
		    	}
		    	else{servoClap.write(CLAPGREEN);}
		    }
		    int limits[RAD_NB_PTS]={0,15};
		    sharpSetLim(limits);
	}

void deinitClap2_YELLOW(sState *next)
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
		        servoClap.write(CLAPNEUTRAL);
		    	}
}

trajElem TrajClap2_YELLOW[]={
	{-500,-7,3000},
	{0,0,0}
};

sState *testClap2_YELLOW()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
			if(periodicProgTraj(TrajClap2_YELLOW,&st_saveTime,&i,&prev_millis)){
				emergencyStop();
				return &sDead;
			}
			if (millis() - _matchStart >= TIME_MATCH_STOP){
				emergencyStop();
				return &sDead;
			 }
	    return 0;
	}

sState sClap2_YELLOW={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initClap2_YELLOW,
        &deinitClap2_YELLOW,
        &testClap2_YELLOW
};


void initClap2_GREEN(sState *prev)
	{

			#ifdef DEBUG
				Serial.println("debut clap2 GREEN");
			#endif
		    if (prev==&sPause)
		    	{
				#ifdef DEBUG
					Serial.println("\tback from pause");
				#endif
		        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
		        _backFromPause=1;
		    	}
		    else{
		    	if (digitalRead(PIN_COLOR)==COLOR_YELLOW){
		    		servoClap.write(CLAPYELLOW);
		    	}
		    	else{servoClap.write(CLAPGREEN);}
		    }
		    int limits[RAD_NB_PTS]={0,15};
		    sharpSetLim(limits);
	}

void deinitClap2_GREEN(sState *next)
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
		        servoClap.write(CLAPNEUTRAL);
		    	}
}

trajElem TrajClap2_GREEN[]={
	{-500,7,3000},
	{0,0,0}
};

sState *testClap2_GREEN()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
			if(periodicProgTraj(TrajClap2_YELLOW,&st_saveTime,&i,&prev_millis)){
				emergencyStop();
				return &sDead;
			}
			if (millis() - _matchStart >= TIME_MATCH_STOP){
				emergencyStop();
				return &sDead;
			 }
	    return 0;
	}

sState sClap2_GREEN={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initClap2_GREEN,
        &deinitClap2_GREEN,
        &testClap2_GREEN
};

