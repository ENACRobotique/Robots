
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
#include "state_Prestairs.h"
#include "state_wait.h"
#include "state_lineMonit.h"

unsigned long st_saveTime=0,st_prevSaveTime=0,TimeToLauncher=0;
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
	    	}
	    else
	    	{
	        st_saveTime=0;
	        st_prevSaveTime=0;
	    	}
	}

trajElem start_green[]={
#ifdef HEADING
				{30,0,5500},
				{0,-90,9000},
				{0,-90,2000},
				{10,-90,2000},
				{0,0,0},
#else
				{30,0,4000},
				{20,-10,3000},
				{15,0,2000},
				{0,0,0},
#endif
				};

sState *testTrajGreenInit()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicFunction(start_green,&st_saveTime,&i,&prev_millis))
	    {

			#ifdef DEBUG
				Serial.println("\tTrajet 1 fini !");
			#endif

	    	 return &sPrestairs;
	    }
	    if (radarIntrusion()) return &sPause;
	    return 0;
	}

sState sTrajGreenInit={
        BIT(E_MOTOR)|BIT(E_RADAR)|BIT(E_HEADING),
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
				{30,0,4500},
				{20,10,6000},
				{15,0,750},
				{0,0,0},
#endif
				};
sState *testTrajYellowInit()
	{
	static int i=0;
		    static unsigned long prev_millis=0;
		    if(periodicFunction(start_yellow,&st_saveTime,&i,&prev_millis))
		   	    {
		    	return &sPrestairs;
		   	    }
		 if (radarIntrusion()) return &sPause;
	    return 0;
	}
sState sTrajYellowInit={
		BIT(E_MOTOR)|BIT(E_RADAR)|BIT(E_HEADING),
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
		        {0,0,0},

		};

sState *testTrajEndStairsYellow()
	{
	static int i=0;
		    static unsigned long prev_millis=0;

			if(periodicFunction(Final_yellow,&st_saveTime,&i,&prev_millis))
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
        {0,0,0},
       };

sState *testTrajEndStairsGreen()
	{
		static int i=0;
	    static unsigned long prev_millis=0;

	    if(periodicFunction(Final_green,&st_saveTime,&i,&prev_millis))
		{
	    	return &sWait;
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
