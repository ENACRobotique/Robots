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
#include "lib_radar2.h"
#include "lib_line.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wall.h"
#include "state_funny.h"
#include "state_wait.h"
#include "state_fresco.h"

unsigned long st_saveTime=0,st_prevSaveTime=0;


void initTrajRedInit(sState *prev)
	{
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++)
    	{
        tab[i]=RADAR_SAFETY_DST;
    	}
    radarSetLim(tab);

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
	}

void deinitTrajRedInit(sState *next)
	{
    //pause
    if (next==&sPause)
    	{
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    	}
    else
    	{
        st_saveTime=0;
        st_prevSaveTime=0;
        //move(0,0);
    	}
}

trajElem start_red[]={ //A MODIFIER
        {0,0,100}, //to wait until the servo is in the good position
        {100,0,3000}, //avant 29cm
        {0,45,500},
        {100,45,750},
        {0,0,500},
        {100,0,750},
        {0,90,500},
        {-100,90,1000},
        {0,0,100},
        {0,0,0}
	};

sState *testTrajRedInit()
	{
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(start_red,&st_saveTime,&i,&prev_millis)) return &sWallLeft;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
	}

sState sTrajRedInit={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initTrajRedInit,
        &deinitTrajRedInit,
        &testTrajRedInit
};




void initTrajYellowInit(sState *prev)
	{
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++)
    	{
        tab[i]=RADAR_SAFETY_DST;
    	}
    radarSetLim(tab);

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
	}

void deinitTrajYellowInit(sState *next)
	{
    //pause
    if (next==&sPause)
    	{
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    	}
    else
    	{
        st_saveTime=0;
        st_prevSaveTime=0;
        //move(0,0);
    	}
}

trajElem start_yellow[]={ //A MODIFIER
        {0,0,100}, //to wait until the servo is in the good position
        {100,0,2800},
        {0,-45,250},
        {50,-45,2000},
        {0,0,250},
        {100,0,500},
        {0,90,250},
        {100,90,800},
        {0,0,100},
        {0,0,0}
	};

sState *testTrajYellowInit()
	{
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(start_yellow,&st_saveTime,&i,&prev_millis)) return &sWallRight;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
	}

sState sTrajYellowInit={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initTrajYellowInit,
        &deinitTrajYellowInit,
        &testTrajYellowInit
		};


//***************Fin trajectoire initiale*****************//



void initTrajRedFinal(sState *prev)
	{
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++)
    	{
        tab[i]=RADAR_SAFETY_DST;
    	}
    radarSetLim(tab);

	#ifdef DEBUG
		Serial.println("debut traj finale rouge");
	#endif

    if (prev==&sPause)
    	{
		#ifdef DEBUG
			Serial.println("\tback from pause");
		#endif
        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
    	}
	}

void deinitTrajRedFinal(sState *next)
	{
    //pause
    if (next==&sPause)
    	{
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    	}
    else
    	{
        st_saveTime=0;
        st_prevSaveTime=0;
        //move(0,0);
    	}
	}

trajElem final_red[]={ //A MODIFIER
        {0,0,100}, //to wait until the servo is in the good position
        {-100,45,650},
        {-100,0,6000},
        {-100,-2,10000},
        {-100,-45,850},
        {-100,-5,10000},
        {0,0,100},
        {0,0,0}
		};

sState *testTrajRedFinal()
	{
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(final_red,&st_saveTime,&i,&prev_millis)) return &sWait;//Cas a voir
    if (lineDetection()) return &sFresco;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
	}

sState sTrajRedFinal={
        BIT(E_MOTOR)| BIT(E_RADAR),
        &initTrajRedFinal,
        &deinitTrajRedFinal,
        &testTrajRedFinal
		};


void initTrajYellowFinal(sState *prev)
	{
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++)
    	{
        tab[i]=RADAR_SAFETY_DST;
    	}
    radarSetLim(tab);

	#ifdef DEBUG
		Serial.println("debut traj finale rouge");
	#endif

    if (prev==&sPause)
    	{
		#ifdef DEBUG
			Serial.println("\tback from pause");
		#endif
        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
    	}
	}

void deinitTrajYellowFinal(sState *next)
	{
    //pause
    if (next==&sPause)
    	{
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    	}
    else
    	{
        st_saveTime=0;
        st_prevSaveTime=0;
        //move(0,0);
    	}
	}

trajElem final_yellow[]={ //A MODIFIER
        {0,0,100}, //to wait until the servo is in the good position
        {-100,-45,1600},
        {-100,0,10000},
        {-100,45,1000},
        {-100,-2,16000},
        {0,0,0}
		};

sState *testTrajYellowFinal()
	{
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(final_yellow,&st_saveTime,&i,&prev_millis)) return &sWait; //Cas a voir
    if (lineDetection()) return &sFresco;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
	}

sState sTrajYellowFinal={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initTrajYellowFinal,
        &deinitTrajYellowFinal,
        &testTrajYellowFinal
		};




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


