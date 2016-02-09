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
	trajElem go_river[] = {
			{400,0,500},
			{600,5,1800},
			{0,0,200},
			{-800,33,1300},
			{-800,5,1900},
			{-800,40,800},
			{-800,3,3000},
			{0,0,100},
			{800,0,1500},
			{0,90,300},
			{650,90,1400},
			{0,0,100},
			{-500,0,4000},
			{0,0,100},
			{800,0,300},
			{0,90,300},
			{650,90,1000},
			{0,0,100},
			{-800,0,2000},
			{-400,0,3000},
			{0,0,10000},
			{0,0,0}
	};
	static unsigned long st_saveTime=0;
	static int i=0;
	static unsigned long prev_millis=0;
	if(periodicProgTraj(go_river,&st_saveTime,&i,&prev_millis))
	{
		return &sWait;
	}
	return NULL;
}

void initPeche(sState *prev){


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

