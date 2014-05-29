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
int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis);
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

#ifdef MUR_R
	{0,0,100}, //to wait until the servo is in the good position
	{-360,0,100},
    {-360,-7,1000},
    {-360,-1,400},
    {-360,0,300},
    {0,0,0},
#endif


#ifdef Int_R
	{0,0,100}, //to wait until the servo is in the good position
	{-360,0,70},
    {-360,-55,1400},
    {-360,-1,600},
    {-360,0,300},
    {0,0,0},
#endif
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
#ifdef MUR_Y
		{0,0,100}, //to wait until the servo is in the good position
		{360,0,320},
		{360,-7,800},
		{360,-5,350},
		{360,-2,200},
	    {0,0,0},
#endif


#ifdef INT_Y
		{0,0,100}, //to wait until the servo is in the good position
		{360,0,470},
		{360,-50,1300},
		{360,-5,350},
		{360,-2,200},
		{0,0,0},
#endif
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
		BIT(E_MOTOR) |BIT(E_RADAR),
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

				{360,0,1900},
				{360,-85,900},
		        {360,-30,500},
		        {0,0,300},

		        {-360,0,100},
		        {-360,-40,1100},
		        {-360,-5,250},
		        {-340,0,700},
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


			if((millis()-st_saveTime)-TimeToLauncher > 3100 ){
				    	 uint16_t limits[RAD_NB_PTS]={3,40};
				    		    		radarSetLim(limits);
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
		{-360,0,900},
		{-360,-60,1000},
        {-360,-5,800},
        {0,0,200},

        {360,0,600},
        {360,-30,1200},
        {360,-1,200},
        {340,-1,300},
        {240,-1,200},
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


	     if((millis()-st_saveTime)-TimeToLauncher > 1700 ){
	    	 uint16_t limits[RAD_NB_PTS]={40,3};
	    		    		radarSetLim(limits);
	     	     				    	 	 }

	if (radarIntrusion()) return &sPause;
	 	    return 0;
	}

sState sTrajRedFinal={
        BIT(E_MOTOR)|BIT(E_RADAR),
        &initTrajRedFinal,
        &deinitTrajRedFinal,
        &testTrajRedFinal
};
//******************************************************************************************************************
int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis){

    if (!(*prev_millis)) *prev_millis=millis();
    move(tab[*i].speed,tab[*i].angle);


    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
    }
    if ( tab[*i].angle==0 && tab[*i].duration==0 && tab[*i].speed==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }

    return 0;
}
