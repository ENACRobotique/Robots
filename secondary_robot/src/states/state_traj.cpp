
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
				{-800,0,1750},
				{-800,-25,2000},
				{-800,8,0500},
				{-800,3,2000},
				//Portes Fermées -> Retour en arrière
				{800,-4,1500},
				{800,0,1000},
				{800,-25,1800},
				{1200,0,3600},//Passage dans la zone de départ
				{800,-16,2500},
				{800,-5,2000},
				{0,0,100},//mini pause ->éviter les sursauts
				{-800,13,1600},
				{-800,-18,500},//petit recallage avant la pêche
				{-800,3,350},
				{0,0,2000},
				//Pêche
				{800,2,1600},
				{800,-25,750},//début évitement
				{800,25,750},
				{800,0,400},
				{800,24,760},
				{800,-22,1200},
				{0,0,100},//fin évitement
				{-800,10,300},//recalage
				{-800,-5,900},
				{0,0,1000},//pause avant marche arrière
				{-800,-25,700},
				{-800,25,700},
				{-800,0,600},
				{-800,25,800},
				{-800,-22,700},
				{-800,0,1500},
				{800,3,900},//retour au point de départ

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

	    	 return &sPrestairs;
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
				{800,2,1600},
				{800,-25,750},//début évitement
				{800,25,750},
				{800,0,400},
				{800,24,760},
				{800,-22,1200},
				{0,0,100},//fin évitement
				{-800,10,300},//recalage
				{-800,-5,900},
				{0,0,1000},//pause avant marche arrière
				{-800,-25,700},
				{-800,25,700},
				{-800,0,600},
				{-800,25,800},
				{-800,-22,700},
				{-800,0,1500},
				{800,3,900},//retour au point de départ

				{0,0,0},
#endif
				};

radarElem start_yellow_radar[]={
				{40,25,5500},
				{0,0,9000},
				{0,0,2000},
				{30,30,3000},
				{0,0,0},
				};
sState *testTrajYellowInit()
	{
	static int i=0;
	static int i_radar=0;
    static unsigned long prev_millis=0;
    static unsigned long prev_millis_radar=0;
		    if(periodicFunction(start_yellow,&st_saveTime,&i,&prev_millis))
		   	    {
				#ifdef DEBUG
					Serial.println("\tTrajet 1 fini !");
				#endif
		    	return &sPrestairs;
		   	    }

		    if(periodicProgRadarLimit(start_yellow_radar,&st_saveTime_radar,&i_radar,&prev_millis_radar)){
		    			#ifdef DEBUG
		    				Serial.println("\tFin radar 1 !");
		    			#endif
		    		}

		 if (radarIntrusion()) return &sPause;
	    return 0;
	}
sState sTrajYellowInit={
		BIT(E_MOTOR)/*|BIT(E_RADAR)|BIT(E_HEADING)*/,
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
        BIT(E_MOTOR)/*|BIT(E_ATTITUDE)*/,
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
        BIT(E_MOTOR)/*|BIT(E_ATTITUDE)*/,
        &initTrajEndStairsGreen,
        &deinitTrajEndStairsGreen,
        &testTrajEndStairsGreen
};
