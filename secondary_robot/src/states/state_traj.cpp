
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_line.h"
#include "lib_wall.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wall.h"
#include "state_wait.h"
#include "state_lineMonit.h"
#include "state_stairs.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,TimeToLauncher=0;
int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);
void initTrajRedInit(sState *prev)
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
				{50,0,3000},
				{40,10,1700},
				{20,0,6000},
				{0,0,0},
				};

sState *testTrajRedInit()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(start_red,&st_saveTime,&i,&prev_millis))
	    {

			#ifdef DEBUG
				Serial.println("\tTrajet 1 fini !");
			#endif

	    	 return &sStairs;
	    }

	    if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajRedInit={
        BIT(E_MOTOR)|BIT(E_RADAR),
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
		{50,0,3000},
		{40,-10,1700},
		{20,0,6000},
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
		    	return &sStairs;
		   	    }
		 if (radarIntrusion()) return &sPause;
	    return 0;
	}
sState sTrajYellowInit={
		BIT(E_MOTOR) |BIT(E_RADAR),
        &initTrajYellowInit,
        &deinitTrajYellowInit,
        &testTrajYellowInit
};


//******************************************************************************************************************
void initTrajEndStairsYellow(sState *prev)
	{
			#ifdef DEBUG
				Serial.println("debut traj jaune");
			#endif
	}

void deinitTrajEndStairsYellow(sState *next)
	{

}

trajElem Final_yellow[]={
		        {0,5,1000},

		};

sState *testTrajEndStairsYellow()
	{
	static int i=0;
		    static unsigned long prev_millis=0;

			if(periodicProgTraj(Final_yellow,&st_saveTime,&i,&prev_millis))
			 	{
				return &sWait;
			 	}
	    return 0;
	}

sState sTrajEndStairsYellow={
        BIT(E_MOTOR),
        &initTrajEndStairsYellow,
        &deinitTrajEndStairsYellow,
        &testTrajEndStairsYellow
};


//******************************************************************************************************************
void initTrajEndStairsGreen(sState *prev)
	{
	  	#ifdef DEBUG
			Serial.println("debut traj vert final");
		#endif
	}

void deinitTrajEndStairsGreen(sState *next)
	{

}

trajElem Final_red[]={
        {0,-5,1000},
       };

sState *testTrajEndStairsGreen()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(Final_red,&st_saveTime,&i,&prev_millis))
		{
	    	return &sWait;
		}

	 	 return 0;
	}

sState sTrajEndStairsGreen={
        BIT(E_MOTOR),
        &initTrajEndStairsGreen,
        &deinitTrajEndStairsGreen,
        &testTrajEndStairsGreen
};
//******************************************************************************************************************
int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis){

    if (!(*prev_millis)) *prev_millis=millis();
    move(tab[*i].speed,tab[*i].omega);


    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
    }
    if ( tab[*i].omega==0 && tab[*i].duration==0 && tab[*i].speed==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }

    return 0;
}
