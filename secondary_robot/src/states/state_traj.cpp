
#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "lib_line.h"
#include "lib_wall.h"
#include "lib_attitude.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_stairs.h"
#include "state_wait.h"
#include "state_lineMonit.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,TimeToLauncher=0;
int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);
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
	    	}
	    uint16_t limits[RAD_NB_PTS]={3,40};
	   	    		radarSetLim(limits);
	}

void deinitTrajGreenInit(sState *next)
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
				{30,0,4500},
				{15,-10,4000},
				{15,0,750},
				{0,0,0},
				};

sState *testTrajGreenInit()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(start_green,&st_saveTime,&i,&prev_millis))
	    {

			#ifdef DEBUG
				Serial.println("\tTrajet 1 fini !");
			#endif

	    	 return &sStairs;
	    }

	    if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajGreenInit={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajGreenInit,
        &deinitTrajGreenInit,
        &testTrajGreenInit
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
			{30,0,4500},
			{15,10,3750},
			{15,0,750},
			{0,0,0},
		};
sState *testTrajYellowInit()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
		    if(periodicProgTraj(start_yellow,&st_saveTime,&i,&prev_millis))
		   	    {
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

void deinitTrajEndStairsYellow(sState *next)
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
				{0,-10,3000},
		        {0,0,0},

		};

sState *testTrajEndStairsYellow()
	{
	static int i=0;
		    static unsigned long prev_millis=0;

			if(periodicProgTraj(Final_yellow,&st_saveTime,&i,&prev_millis))
			 	{
				return &sWait;
			 	}

//
//			if((millis()-st_saveTime)-TimeToLauncher > 3000 ){
//				    	 uint16_t limits[RAD_NB_PTS]={3,40};
//				    		    		radarSetLim(limits);
//				   	     	}
//
//		if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajEndStairsYellow={
        BIT(E_MOTOR)|BIT(E_ATTITUDE),
        &initTrajEndStairsYellow,
        &deinitTrajEndStairsYellow,
        &testTrajEndStairsYellow
};


//******************************************************************************************************************
void initTrajEndStairsGreen(sState *prev)
	{
	  	#ifdef DEBUG
			Serial.println("debut traj rouge final");
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

void deinitTrajEndStairsGreen(sState *next)
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
//////////////////////////////////////////////////////////////////////////////////////////////////////
trajElem Final_green[]={
		{0,10,3000},
        {0,0,0},
       };

sState *testTrajEndStairsGreen()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicProgTraj(Final_green,&st_saveTime,&i,&prev_millis))
		{
	    	return &sWait;
		}


	    if((millis()-st_saveTime)-TimeToLauncher > 1500 && TimeToLauncher!=0 ){
	    	launcherServoDown.write(15);
	    	TimeToLauncher=0;
		}


//	     if((millis()-st_saveTime)-TimeToLauncher > 3100 ){
//	    	 uint16_t limits[RAD_NB_PTS]={40,3};
//	    		    		radarSetLim(limits);
//	     	     				    	 	 }
//
//	if (radarIntrusion()) return &sPause;
	 	    return 0;
	}

sState sTrajEndStairsGreen={
        BIT(E_MOTOR)|BIT(E_ATTITUDE),
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
