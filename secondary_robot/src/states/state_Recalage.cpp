/*
 * state_Recalage.cpp
 *
 *  Created on: 2016 février 05
 *      Author: Darian
 */


#include "Arduino.h"
#include "state_Recalage.h"
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
#include "state_wait.h"
#include "state_lineMonit.h"
#include "lib_radar_mask.h"
#include "state_Peche.h"


sState* testRecalage(){
        trajElem calage_largeur[] = {
				{800,0,500},
				{0,-90,300},
				{650,-90,1200},
				{0,0,100},
				{-800,0,7000},
				{-400,0,15000},
				{0,0,0}
        };
        static unsigned long st_saveTime=0;
        static int i=0;
		static int i_radar=0;
		static unsigned long prev_millis=0;
		static unsigned long prev_millis_radar=0;
		if(periodicProgTraj(calage_largeur,&st_saveTime,&i,&prev_millis))
			{
				return NULL;
			}
		if(digitalRead(PIN_SWITCH_RIGHT) && (i>=1))
			{
				move(0,0);
				return &sPeche;
			}

	}

void initRecalage(sState *prev){

    }

void deinitRecalage(sState *next){
        // Your code here !
    }

sState sRecalage={
	BIT(E_MOTOR),
    &initRecalage,
    &deinitRecalage,
    &testRecalage
};

