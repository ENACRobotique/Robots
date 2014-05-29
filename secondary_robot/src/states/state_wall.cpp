
#include "state_wall.h"
#include "state_pause.h"
#include "state_funny.h"
#include "state_traj.h"

#include "../params.h"
#include "../tools.h"
#include "lib_radar2.h"
#include "lib_wall.h"
#include "lib_line.h"
#include "lib_move.h"



unsigned long int savetime=0;


void initWallLeft(sState *prev)
	{
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++)
    	{
        tab[i]=RADAR_SAFETY_DST;
    	}
    radarSetLim(tab);

	#ifdef DEBUG
		Serial.println("debut wall left");
	#endif

    wallSetVal(LEFT,WALL_DST,WALL_SPEED);

    if (prev==&sPause)
    	{
		#ifdef DEBUG
    		Serial.println("\tback from pause");
		#endif
        savetime=millis()-savetime;
    	}
    else
    	{
    	//launcherServoDown.write(LAUNCHER_DOWN_POS_0);
    	//launcherServoUp.write(LAUNCHER_UP_POS_0);
    	}
    }


void deinitWallLeft(sState *next)
	{
    //pause
    if (next==&sPause) {
        savetime=millis();
    }
    else
    	{
        savetime=0;
        //move(0,0);
    	}
	}


sState * testWallLeft()
	{
    static unsigned long prevMillisWallLeft=millis();

  //  if((millis() > prevMillisWallLeft + TIME_READY_LAUNCHER) && (diff(PIN_SHARP_FRONT_LEFT, PIN_SHARP_BACK_LEFT)<1))
    	//{
    	//launcherServoDown.write(LAUNCHER_DOWN_POS_3);
    	//launcherServoUp.write(LAUNCHER_UP_POS_3);
    //	}

   // if(endWall(PIN_SHARP_BACK_RIGHT, PIN_SHARP_FRONT_RIGHT)) return &sTrajRedFinal;


    //if(radarIntrusion()) return &sPause;
    return 0;
	}

sState sWallLeft={
        BIT(E_MOTOR) | BIT(E_RADAR) | BIT(E_WALL),
        &initWallLeft,
        &deinitWallLeft,
        &testWallLeft
		};



//right


void initWallRight(sState *prev)
	{
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++)
    	{
        tab[i]=RADAR_SAFETY_DST;
    	}
    radarSetLim(tab);

	#ifdef DEBUG
		Serial.println("debut wall right");
	#endif

    wallSetVal(RIGHT,WALL_DST,-WALL_SPEED);

    if (prev==&sPause)
    	{
		#ifdef DEBUG
			Serial.println("\tback from pause");
		#endif
        savetime=millis()-savetime;
    	}
	}

void deinitWallRight(sState *next)
	{
    //pause
    if (next==&sPause)
    	{
        savetime=millis();
    	}
    else
    	{
        savetime=0;
        move(0,0);
    	}
	}


sState * testWallRight()
	{
    static unsigned long prevMillisWallLeft=millis();

   // if((millis() > prevMillisWallLeft + TIME_READY_LAUNCHER) && (diff(PIN_SHARP_FRONT_LEFT, PIN_SHARP_BACK_LEFT)<1))
    //	{
    	//launcherServoDown.write(LAUNCHER_DOWN_POS_3);
    	//launcherServoUp.write(LAUNCHER_UP_POS_3);
    //	}
   // if(endWall(PIN_SHARP_FRONT_RIGHT, PIN_SHARP_BACK_RIGHT)) return &sTrajRedFinal;
   // if(radarIntrusion()) return &sPause;
    return 0;
	}

sState sWallRight={
        BIT(E_MOTOR) | BIT(E_RADAR) | BIT(E_WALL),
        &initWallRight,
        &deinitWallRight,
        &testWallRight
};








