/*
 * state_Peche.cpp
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


sState* testPeche(){
        // Your code here !
        return NULL;
    }

void initPeche(sState *prev){
	trajElem go_river[] = {
					{800,0,2000},
					{0,0,10000},
					{0,0,0}
	        };
	        static unsigned long st_saveTime=0;
	        static int i=0;
			static int i_radar=0;
			static unsigned long prev_millis=0;
			static unsigned long prev_millis_radar=0;
			periodicProgTraj(go_river,&st_saveTime,&i,&prev_millis);

    }

void deinitPeche(sState *next){
        // Your code here !
    }

sState sPeche={
	BIT(E_MOTOR),
        &initPeche,
        &deinitPeche,
        &testPeche
};

